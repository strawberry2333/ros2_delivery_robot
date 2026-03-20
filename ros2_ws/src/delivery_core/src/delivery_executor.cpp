/**
 * @file delivery_executor.cpp
 * @brief 配送执行器生命周期节点实现。
 *
 * 这个文件实现的是“单订单执行层”，不负责队列调度，也不直接接收用户订单。
 * 它的职责是把 manager 发来的 ExecuteDelivery goal，转化为一次真实的行为树执行，
 * 并在执行过程中与 Nav2、装/卸货确认服务、状态话题保持联动。
 *
 * 生命周期安排的意义在于把“系统准备”和“对外交付能力”分开：
 * - configure：加载站点、创建 BT 依赖、注册节点
 * - activate：确认 Nav2 可用后再开放 action server
 * - deactivate/cleanup/shutdown：回收 BT 线程、辅助节点和执行状态
 */

#include "delivery_core/delivery_executor.hpp"

#include "delivery_core/nodes/check_battery.hpp"
#include "delivery_core/nodes/dock_at_station.hpp"
#include "delivery_core/nodes/navigate_to_station.hpp"
#include "delivery_core/nodes/report_delivery_status.hpp"
#include "delivery_core/nodes/wait_for_confirmation.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "yaml-cpp/yaml.h"

#include <algorithm>
#include <chrono>
#include <functional>
#include <future>
#include <thread>
#include <vector>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace delivery_core
{

DeliveryExecutor::DeliveryExecutor()
: LifecycleNode("delivery_executor")
{
  // 构造阶段只做参数声明，不做重资源初始化。
  // 这样可以让 lifecycle manager 先把节点实例拉起来，再决定何时真正配置。
  station_config_path_ = this->declare_parameter<std::string>("station_config", "");
  tree_file_path_ = this->declare_parameter<std::string>("tree_file", "");
  battery_drain_per_delivery_ = this->declare_parameter<double>(
    "battery_drain_per_delivery", 15.0);

  RCLCPP_INFO(get_logger(), "DeliveryExecutor 已创建 (Unconfigured)");
}

DeliveryExecutor::~DeliveryExecutor()
{
  // 析构时不依赖外部生命周期状态，直接尝试把所有后台资源收口。
  request_bt_stop();
  join_bt_thread();
  stop_bt_helper_node();
}

// ======================== 生命周期回调 ========================

DeliveryExecutor::CallbackReturn DeliveryExecutor::on_configure(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "on_configure: 开始配置...");
  stop_requested_.store(false, std::memory_order_release);

  // 将服务回调与其他回调隔离开，避免在 BT 执行或关闭阶段出现互相阻塞。
  service_cb_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::Reentrant);

  // BT 叶节点需要普通 rclcpp::Node 接口来创建 action client / publisher。
  // 但 LifecycleNode 本身不是 Node，所以这里创建一个轻量辅助节点专门给 BT 使用。
  // 禁用全局参数重映射，避免外部 __node:=... 影响该辅助节点的命名和参数行为。
  bool use_sim_time = false;
  (void)this->get_parameter("use_sim_time", use_sim_time);
  rclcpp::NodeOptions bt_node_options;
  bt_node_options.use_global_arguments(false);
  bt_node_options.automatically_declare_parameters_from_overrides(true);
  bt_node_options.parameter_overrides(
    std::vector<rclcpp::Parameter>{
      rclcpp::Parameter("use_sim_time", use_sim_time)});
  bt_node_ = rclcpp::Node::make_shared(
    "delivery_executor_bt_helper",
    this->get_namespace(),
    bt_node_options);

  // Nav2 导航是 BT 的核心外部依赖。
  // 这里用辅助节点创建 action client，确保导航请求和 action 回调都能正常处理。
  nav_client_ = rclcpp_action::create_client<NavigateToPose>(
    bt_node_, "navigate_to_pose");

  // 辅助节点本身也需要 spin，否则 action client 的异步回调无法被处理。
  // 单独开线程可以避免把 BT 执行和回调处理绑死在同一个执行上下文里。
  bt_node_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  bt_node_executor_->add_node(bt_node_);
  bt_node_spin_thread_ = std::thread([this]() {
        bt_node_executor_->spin();
      });

  // 状态话题用于向 manager / 外部监控暴露执行进度。
  status_pub_ = this->create_publisher<DeliveryStatus>("delivery_status", 10);
  // 停靠节点会通过速度话题对机器人做微调运动。
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  // 装货确认服务：人或上位机调用后，把 load_confirmed_ 置位。
  confirm_load_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "confirm_load",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      load_confirmed_.store(true, std::memory_order_release);
      response->success = true;
      response->message = "装货确认已接收";
      RCLCPP_INFO(get_logger(), "收到装货确认信号");
    },
    rclcpp::ServicesQoS(), service_cb_group_);

  // 卸货确认服务：与装货确认同理，但对应订单后半段的人工确认。
  confirm_unload_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "confirm_unload",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      unload_confirmed_.store(true, std::memory_order_release);
      response->success = true;
      response->message = "卸货确认已接收";
      RCLCPP_INFO(get_logger(), "收到卸货确认信号");
    },
    rclcpp::ServicesQoS(), service_cb_group_);

  // 站点配置是 executor 判断订单合法性和给 BT 提供环境上下文的基础。
  if (!load_station_config(station_config_path_)) {
    RCLCPP_ERROR(get_logger(), "站点配置加载失败");
    return CallbackReturn::FAILURE;
  }

  // 站点加载成功后，再把 BT 节点注册进 factory。
  // 这样 XML 里引用这些节点时，构建行为树才不会失败。
  register_bt_nodes();

  RCLCPP_INFO(get_logger(),
    "on_configure: 完成，已加载 %zu 个站点", stations_.size());
  return CallbackReturn::SUCCESS;
}

DeliveryExecutor::CallbackReturn DeliveryExecutor::on_activate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "on_activate: 等待 Nav2 navigate_to_pose action server...");
  stop_requested_.store(false, std::memory_order_release);

  // 先确认底层导航栈可用，再对外开放 ExecuteDelivery。
  // 这样 manager 看到的“可接单”，才是真的可以执行，而不是假就绪。
  if (!nav_client_->wait_for_action_server(std::chrono::seconds(30))) {
    RCLCPP_ERROR(get_logger(), "on_activate: Nav2 navigate_to_pose 不可用，激活失败");
    return CallbackReturn::FAILURE;
  }
  if (!wait_for_nav2_active(std::chrono::seconds(60))) {
    RCLCPP_ERROR(get_logger(), "on_activate: bt_navigator 未进入 Active，激活失败");
    return CallbackReturn::FAILURE;
  }
  RCLCPP_INFO(get_logger(), "on_activate: Nav2 已 Active，创建 Action Server...");

  // 只有当 Nav2 和 bt_navigator 都已经 ready，才创建对外 action server。
  action_server_ = rclcpp_action::create_server<ExecuteDelivery>(
    this,
    "execute_delivery",
    std::bind(&DeliveryExecutor::handle_goal, this, _1, _2),
    std::bind(&DeliveryExecutor::handle_cancel, this, _1),
    std::bind(&DeliveryExecutor::handle_accepted, this, _1),
    rcl_action_server_get_default_options(),
    service_cb_group_);

  RCLCPP_INFO(get_logger(), "on_activate: Action Server 就绪");
  return CallbackReturn::SUCCESS;
}

DeliveryExecutor::CallbackReturn DeliveryExecutor::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  // 停用阶段优先收敛执行线程，再销毁 action server。
  // 这样可以避免“goal 还在跑，但服务已经消失”的半关闭状态。
  RCLCPP_INFO(get_logger(), "on_deactivate: 停止服务...");
  request_bt_stop();
  join_bt_thread();
  goal_inflight_.store(false, std::memory_order_release);
  action_server_.reset();

  RCLCPP_INFO(get_logger(), "on_deactivate: 完成");
  return CallbackReturn::SUCCESS;
}

DeliveryExecutor::CallbackReturn DeliveryExecutor::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  // cleanup 比 deactivate 更彻底，适合把节点恢复到可重新 configure 的初始状态。
  RCLCPP_INFO(get_logger(), "on_cleanup: 清理资源...");
  request_bt_stop();
  join_bt_thread();
  goal_inflight_.store(false, std::memory_order_release);
  action_server_.reset();
  stop_bt_helper_node();

  stations_.clear();
  bt_node_.reset();
  nav_client_.reset();
  status_pub_.reset();
  cmd_vel_pub_.reset();
  confirm_load_srv_.reset();
  confirm_unload_srv_.reset();
  battery_level_.store(100.0);

  // 反注册所有 BT 节点，避免下次 configure 时重复注册同名 builder。
  factory_.unregisterBuilder("NavigateToStation");
  factory_.unregisterBuilder("DockAtStation");
  factory_.unregisterBuilder("WaitForConfirmation");
  factory_.unregisterBuilder("ReportDeliveryStatus");
  factory_.unregisterBuilder("CheckBattery");

  RCLCPP_INFO(get_logger(), "on_cleanup: 完成");
  return CallbackReturn::SUCCESS;
}

DeliveryExecutor::CallbackReturn DeliveryExecutor::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  // shutdown 是进程级退出的最后收口，不再追求重启友好，只追求干净退出。
  RCLCPP_INFO(get_logger(), "on_shutdown: 关闭节点");
  request_bt_stop();
  join_bt_thread();
  goal_inflight_.store(false, std::memory_order_release);
  action_server_.reset();
  stop_bt_helper_node();

  return CallbackReturn::SUCCESS;
}

void DeliveryExecutor::request_bt_stop()
{
  // 只设置停止标志，真正的收尾交给 BT tick 循环和 join 逻辑处理。
  stop_requested_.store(true, std::memory_order_release);
}

void DeliveryExecutor::join_bt_thread()
{
  std::thread thread_to_join;
  {
    std::lock_guard<std::mutex> lock(execution_mutex_);
    if (bt_execution_thread_.joinable()) {
      // 先把线程对象移出临界区，再在锁外 join，避免阻塞其他需要同一把锁的收尾逻辑。
      thread_to_join = std::move(bt_execution_thread_);
    }
  }

  if (thread_to_join.joinable()) {
    thread_to_join.join();
  }

  std::lock_guard<std::mutex> lock(execution_mutex_);
  active_goal_handle_.reset();
}

void DeliveryExecutor::stop_bt_helper_node()
{
  // 辅助节点有自己的 executor / spin 线程，需要显式 cancel + join。
  if (bt_node_executor_) {
    bt_node_executor_->cancel();
  }
  if (bt_node_spin_thread_.joinable()) {
    bt_node_spin_thread_.join();
  }
  if (bt_node_executor_ && bt_node_) {
    bt_node_executor_->remove_node(bt_node_);
  }
  bt_node_executor_.reset();
}

bool DeliveryExecutor::wait_for_nav2_active(std::chrono::seconds timeout)
{
  using GetState = lifecycle_msgs::srv::GetState;

  // bt_navigator 的生命周期状态要通过 get_state 服务来确认。
  // 这里不只等 action server，还要确认导航栈真正进入 Active。
  auto state_client = bt_node_->create_client<GetState>("/bt_navigator/get_state");
  if (!state_client->wait_for_service(timeout)) {
    RCLCPP_ERROR(get_logger(), "等待 /bt_navigator/get_state 服务超时");
    return false;
  }

  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    // 周期性查询生命周期状态，避免一次性请求卡死整个启动流程。
    auto future = state_client->async_send_request(std::make_shared<GetState::Request>());
    if (future.wait_for(1s) != std::future_status::ready) {
      rclcpp::sleep_for(500ms);
      continue;
    }

    const auto response = future.get();
    if (!response) {
      rclcpp::sleep_for(500ms);
      continue;
    }

    if (response->current_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      RCLCPP_INFO(get_logger(), "bt_navigator 已进入 Active");
      return true;
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "等待 bt_navigator 进入 Active，当前状态=%s (%u)",
      response->current_state.label.c_str(),
      response->current_state.id);
    rclcpp::sleep_for(500ms);
  }

  return false;
}

void DeliveryExecutor::clear_active_goal(
  const std::shared_ptr<GoalHandleExecuteDelivery> & goal_handle)
{
  // 只在当前线程持有的 goal 仍然是 active_goal_handle_ 时才清理，避免误删新任务。
  std::lock_guard<std::mutex> lock(execution_mutex_);
  if (active_goal_handle_ == goal_handle) {
    active_goal_handle_.reset();
  }
}

// ======================== 配置加载 ========================

bool DeliveryExecutor::load_station_config(const std::string & path)
{
  if (path.empty()) {
    RCLCPP_ERROR(get_logger(), "站点配置路径为空");
    return false;
  }

  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "YAML 解析错误: %s", e.what());
    return false;
  }

  const auto stations_node = root["stations"];
  if (!stations_node || !stations_node.IsSequence()) {
    RCLCPP_ERROR(get_logger(), "YAML 中缺少 'stations' 列表");
    return false;
  }

  // 站点表重建前先清空旧数据，避免重复 configure 时残留上一轮内容。
  stations_.clear();
  for (size_t i = 0; i < stations_node.size(); ++i) {
    const auto & node = stations_node[i];
    Station station;
    station.id = node["station_id"].as<std::string>();
    station.pose.x = node["x"].as<double>();
    station.pose.y = node["y"].as<double>();
    station.pose.yaw = node["yaw"] ? node["yaw"].as<double>() : 0.0;
    station.type = node["station_type"] ? node["station_type"].as<uint8_t>() : 0;

    if (station.type > 2) {
      RCLCPP_ERROR(get_logger(), "站点 [%s] 类型非法: %u",
                         station.id.c_str(), station.type);
      return false;
    }

    // 站点 ID 必须唯一，否则 BT 和 manager 都无法稳定地根据 ID 找到目标位置。
    if (stations_.count(station.id) > 0) {
      RCLCPP_ERROR(get_logger(), "站点 ID 重复: %s", station.id.c_str());
      return false;
    }

    stations_[station.id] = station;
    RCLCPP_INFO(get_logger(), "  站点 [%s]: (%.2f, %.2f, %.2f) 类型=%u",
                    station.id.c_str(), station.pose.x, station.pose.y,
                    station.pose.yaw, station.type);
  }

  return !stations_.empty();
}

// ======================== BT 工厂初始化 ========================

void DeliveryExecutor::register_bt_nodes()
{
    // 使用辅助节点供 BT 叶节点使用。
    // 这些节点本质上都是“把业务状态翻译成机器人动作”的适配器。
  auto node_ptr = bt_node_;

    // NavigateToStation：把目标站点翻译成 Nav2 的 navigate_to_pose 请求。
  factory_.registerBuilder<NavigateToStation>(
        "NavigateToStation",
    [node_ptr, this](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<NavigateToStation>(
                name, config, node_ptr, nav_client_);
        });

    // DockAtStation：在到站后做最后一点平面内微调，让机器人更贴近站点。
  factory_.registerBuilder<DockAtStation>(
        "DockAtStation",
    [node_ptr, this](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<DockAtStation>(
                name, config, node_ptr, cmd_vel_pub_);
        });

    // WaitForConfirmation：把人工确认从“外部服务调用”转成 BT 里的等待条件。
  factory_.registerBuilder<WaitForConfirmation>(
        "WaitForConfirmation",
    [node_ptr, this](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<WaitForConfirmation>(
                name, config, node_ptr, &load_confirmed_, &unload_confirmed_);
        });

    // ReportDeliveryStatus：统一更新状态话题和黑板上的执行进度。
  factory_.registerBuilder<ReportDeliveryStatus>(
        "ReportDeliveryStatus",
    [node_ptr, this](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<ReportDeliveryStatus>(
                name, config, node_ptr, status_pub_);
        });

    // CheckBattery：在任务开始前读取黑板中的电量，决定是否允许继续配送。
  factory_.registerNodeType<CheckBattery>("CheckBattery");
}

// ======================== Action Server 回调 ========================

rclcpp_action::GoalResponse DeliveryExecutor::handle_goal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const ExecuteDelivery::Goal> goal)
{
  const auto & order = goal->order;

  // 这里的职责是“接单前过滤”，不是实际执行。
  RCLCPP_INFO(get_logger(), "收到配送请求 [%s]: %s → %s",
                order.order_id.c_str(),
                order.pickup_station.c_str(),
                order.dropoff_station.c_str());

    // 先校验订单引用的站点是否都存在，否则后续 BT 无法解析目标。
  if (stations_.find(order.pickup_station) == stations_.end() ||
    stations_.find(order.dropoff_station) == stations_.end())
  {
    RCLCPP_WARN(get_logger(), "拒绝配送请求 [%s]：站点不存在",
                    order.order_id.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

    // 单任务模式：在 goal 接收阶段就预留执行槽位，避免和 handle_accepted 之间出现竞态窗口。
  bool expected = false;
  if (!goal_inflight_.compare_exchange_strong(
      expected, true, std::memory_order_acq_rel, std::memory_order_acquire))
  {
    RCLCPP_WARN(get_logger(), "拒绝配送请求 [%s]：当前已有任务在执行或等待启动",
                    order.order_id.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DeliveryExecutor::handle_cancel(
  const std::shared_ptr<GoalHandleExecuteDelivery>)
{
  // action 层统一接受取消请求，真正的停止动作由 BT tick 循环感知 stop/cancel 后完成。
  RCLCPP_INFO(get_logger(), "收到取消配送请求");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void DeliveryExecutor::handle_accepted(
  const std::shared_ptr<GoalHandleExecuteDelivery> goal_handle)
{
  // 为了保持单任务语义，先等待上一次执行线程彻底退出。
  join_bt_thread();

  stop_requested_.store(false, std::memory_order_release);
  {
    std::lock_guard<std::mutex> lock(execution_mutex_);
    active_goal_handle_ = goal_handle;
  }

  // BT tick 是长耗时流程，必须放到独立线程，避免阻塞 action server 回调。
  bt_execution_thread_ = std::thread([this, goal_handle]() {
        execute_bt(goal_handle);
      });
}

// ======================== BT 执行核心循环 ========================

void DeliveryExecutor::execute_bt(
  const std::shared_ptr<GoalHandleExecuteDelivery> goal_handle)
{
  // 进入执行期后，标记当前节点正在处理一单配送。
  executing_.store(true);

  const auto & order = goal_handle->get_goal()->order;
  const auto start_time = std::chrono::steady_clock::now();
  const auto finish_execution = [this, &goal_handle]() {
      // 收尾动作统一放在 lambda 中，避免在多个 return 分支里重复写一致的清理逻辑。
      executing_.store(false, std::memory_order_release);
      goal_inflight_.store(false, std::memory_order_release);
      clear_active_goal(goal_handle);
    };

  RCLCPP_INFO(get_logger(), "开始执行配送 [%s]", order.order_id.c_str());

  // 每单开始前先清零确认标志，避免上一单的人工确认残留到下一单。
  load_confirmed_.store(false, std::memory_order_release);
  unload_confirmed_.store(false, std::memory_order_release);

  // 根据 tree_file_path_ 生成本单的行为树实例。
  // 这意味着单次任务的策略不是写死在 C++ 里，而是由 XML 决定。
  BT::Tree tree;
  try {
    tree = factory_.createTreeFromFile(tree_file_path_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "创建行为树失败: %s", e.what());

    auto result = std::make_shared<ExecuteDelivery::Result>();
    result->success = false;
    result->error_msg = std::string("BT 创建失败: ") + e.what();

    if (!stop_requested_.load(std::memory_order_acquire) && goal_handle->is_active()) {
      try {
        goal_handle->abort(result);
      } catch (const std::exception & ex) {
        RCLCPP_WARN(get_logger(), "发布 BT 创建失败结果时异常: %s", ex.what());
      }
    }
    finish_execution();
    return;
  }

  // 黑板是 BT 节点之间共享上下文的核心。
  // 这里把订单信息、站点表和当前电量一次性写进去，供子节点读取。
  tree.rootBlackboard()->set("order_id", order.order_id);
  tree.rootBlackboard()->set("pickup_station", order.pickup_station);
  tree.rootBlackboard()->set("dropoff_station", order.dropoff_station);
  tree.rootBlackboard()->set("stations", stations_);
  tree.rootBlackboard()->set("battery_level", battery_level_.load());

  // 在真正进入 tick 之前，先向外发布一个“已开始前往取货点”的状态，
  // 这样 manager 和监控端不会在执行初期看到空白。
  {
    DeliveryStatus status_msg;
    status_msg.stamp = this->now();
    status_msg.order_id = order.order_id;
    status_msg.state = DeliveryStatus::STATE_GOING_TO_PICKUP;
    status_msg.current_station = order.pickup_station;
    status_msg.progress = 0.1f;
    status_pub_->publish(status_msg);
  }

  // BT tick 循环是整个执行过程的主驱动器。
  // 每次 tick 都可能推进状态、发起导航、等待确认或结束任务。
  BT::NodeStatus bt_status = BT::NodeStatus::RUNNING;
  rclcpp::Rate rate(100);  // 100Hz tick

  while (rclcpp::ok() && bt_status == BT::NodeStatus::RUNNING) {
    // 收到外部停机请求时，优先终止本单执行并退出循环。
    if (stop_requested_.load(std::memory_order_acquire)) {
      RCLCPP_INFO(get_logger(), "停止当前配送执行 [%s]", order.order_id.c_str());
      tree.haltTree();
      finish_execution();
      return;
    }

    // 取消请求来自 action 层。这里进入真正的任务收尾分支。
    if (goal_handle->is_canceling()) {
      RCLCPP_INFO(get_logger(), "配送被取消 [%s]", order.order_id.c_str());
      tree.haltTree();

      auto result = std::make_shared<ExecuteDelivery::Result>();
      result->success = false;
      result->error_msg = "配送被取消";

      const auto elapsed = std::chrono::steady_clock::now() - start_time;
      result->elapsed_time_sec =
        std::chrono::duration_cast<std::chrono::duration<double>>(elapsed).count();

      if (!stop_requested_.load(std::memory_order_acquire) && goal_handle->is_active()) {
        try {
          goal_handle->canceled(result);
        } catch (const std::exception & ex) {
          RCLCPP_WARN(get_logger(), "发布取消结果时异常: %s", ex.what());
        }
      }
      finish_execution();
      return;
    }

    // 每个周期只 tick 一次，避免把整个线程长时间卡死在单个节点内部。
    bt_status = tree.tickOnce();

    if (stop_requested_.load(std::memory_order_acquire)) {
      RCLCPP_INFO(get_logger(), "停止当前配送执行 [%s]", order.order_id.c_str());
      tree.haltTree();
      finish_execution();
      return;
    }

    // 从黑板读取最新状态，转成 action feedback 给 manager。
    // 这样外部可以看到细粒度进度，而不只是最终成功/失败。
    auto feedback = std::make_shared<ExecuteDelivery::Feedback>();
    auto bb = tree.rootBlackboard();
    unsigned bt_state = DeliveryStatus::STATE_GOING_TO_PICKUP;
    (void)bb->get("bt_state", bt_state);
    feedback->state = static_cast<uint8_t>(bt_state);
    std::string bt_station;
    (void)bb->get("bt_station", bt_station);
    feedback->current_station = bt_station;
    float prog = 0.0f;
    (void)bb->get("bt_progress", prog);
    feedback->progress = prog;
    if (goal_handle->is_active()) {
      try {
        goal_handle->publish_feedback(feedback);
      } catch (const std::exception & ex) {
        RCLCPP_WARN(get_logger(), "发布配送反馈时异常: %s", ex.what());
      }
    }

    // 100Hz tick 既足够平滑，又不会把 CPU 占满。
    rate.sleep();
  }

  if (stop_requested_.load(std::memory_order_acquire) || !rclcpp::ok()) {
    tree.haltTree();
    finish_execution();
    return;
  }

  // 行为树退出 RUNNING 后，根据最终状态构造 action result。
  auto result = std::make_shared<ExecuteDelivery::Result>();
  const auto elapsed = std::chrono::steady_clock::now() - start_time;
  result->elapsed_time_sec =
    std::chrono::duration_cast<std::chrono::duration<double>>(elapsed).count();

  if (bt_status == BT::NodeStatus::SUCCESS) {
    result->success = true;

        // 配送完成后扣减模拟电量，作为 demo 级的任务消耗模型。
    double current = battery_level_.load();
    double updated = std::max(0.0, current - battery_drain_per_delivery_);
    battery_level_.store(updated);
    RCLCPP_INFO(get_logger(), "配送完成 [%s]，耗时 %.1f 秒，剩余电量 %.1f%%",
      order.order_id.c_str(), result->elapsed_time_sec,
      updated);
    if (goal_handle->is_active()) {
      try {
        goal_handle->succeed(result);
      } catch (const std::exception & ex) {
        RCLCPP_WARN(get_logger(), "发布成功结果时异常: %s", ex.what());
      }
    }
  } else {
    result->success = false;
    double current_battery = battery_level_.load();
    // 失败原因区分“电量不足”与“BT 逻辑失败”，便于排查是系统约束还是流程错误。
    if (current_battery < 20.0) {
      result->error_msg = "电量不足 (" +
        std::to_string(static_cast<int>(current_battery)) + "%)，配送中止";
    } else {
      result->error_msg = "行为树执行失败";
    }
    RCLCPP_WARN(get_logger(), "配送失败 [%s]: %s",
      order.order_id.c_str(), result->error_msg.c_str());

    // 将失败同步到状态话题，让 manager / 监控端能直接看到终态。
    DeliveryStatus fail_msg;
    fail_msg.stamp = this->now();
    fail_msg.order_id = order.order_id;
    fail_msg.state = DeliveryStatus::STATE_FAILED;
    fail_msg.error_msg = result->error_msg;
    status_pub_->publish(fail_msg);

    if (goal_handle->is_active()) {
      try {
        goal_handle->abort(result);
      } catch (const std::exception & ex) {
        RCLCPP_WARN(get_logger(), "发布失败结果时异常: %s", ex.what());
      }
    }
  }

  finish_execution();
}

}  // namespace delivery_core

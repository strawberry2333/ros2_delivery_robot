/**
 * @file delivery_executor.cpp
 * @brief 配送执行器生命周期节点实现。
 *
 * 已实现为 LifecycleNode。
 * - 构造函数：仅声明参数
 * - on_configure：加载站点配置、注册 BT 节点、创建 publisher/service
 * - on_activate：创建 Action Server
 * - on_deactivate：停止 BT、销毁 Action Server
 * - on_cleanup：清除站点数据
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
  // 仅声明参数，实际初始化在 on_configure 中完成
  station_config_path_ = this->declare_parameter<std::string>("station_config", "");
  tree_file_path_ = this->declare_parameter<std::string>("tree_file", "");
  battery_drain_per_delivery_ = this->declare_parameter<double>(
    "battery_drain_per_delivery", 15.0);

  RCLCPP_INFO(get_logger(), "DeliveryExecutor 已创建 (Unconfigured)");
}

DeliveryExecutor::~DeliveryExecutor()
{
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

  // 创建回调组
  service_cb_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::Reentrant);

  // 创建辅助 Node 供 BT 节点使用（LifecycleNode 不继承 rclcpp::Node）。
  // 显式禁用全局参数重映射，避免 __node:=delivery_executor 污染辅助节点名称。
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

  // 创建 Nav2 Action Client（通过辅助节点）
  nav_client_ = rclcpp_action::create_client<NavigateToPose>(
    bt_node_, "navigate_to_pose");

  // 启动辅助节点的 executor spin 线程，确保 Nav2 action client 回调能被处理
  bt_node_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  bt_node_executor_->add_node(bt_node_);
  bt_node_spin_thread_ = std::thread([this]() {
        bt_node_executor_->spin();
      });

  // 创建 Publisher
  status_pub_ = this->create_publisher<DeliveryStatus>("delivery_status", 10);
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  // 创建确认服务端
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

  // 加载站点配置
  if (!load_station_config(station_config_path_)) {
    RCLCPP_ERROR(get_logger(), "站点配置加载失败");
    return CallbackReturn::FAILURE;
  }

  // 注册 BT 节点
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

  // 在暴露 ExecuteDelivery Action Server 之前，确保 Nav2 已就绪
  // 这保证 manager 等到 executor 可用时 Nav2 也一定可用
  if (!nav_client_->wait_for_action_server(std::chrono::seconds(30))) {
    RCLCPP_ERROR(get_logger(), "on_activate: Nav2 navigate_to_pose 不可用，激活失败");
    return CallbackReturn::FAILURE;
  }
  if (!wait_for_nav2_active(std::chrono::seconds(60))) {
    RCLCPP_ERROR(get_logger(), "on_activate: bt_navigator 未进入 Active，激活失败");
    return CallbackReturn::FAILURE;
  }
  RCLCPP_INFO(get_logger(), "on_activate: Nav2 已 Active，创建 Action Server...");

  // 创建 ExecuteDelivery Action Server
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

  // 反注册所有 BT 节点，避免二次 configure 时 "ID already registered" 错误
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
  stop_requested_.store(true, std::memory_order_release);
}

void DeliveryExecutor::join_bt_thread()
{
  std::thread thread_to_join;
  {
    std::lock_guard<std::mutex> lock(execution_mutex_);
    if (bt_execution_thread_.joinable()) {
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

  auto state_client = bt_node_->create_client<GetState>("/bt_navigator/get_state");
  if (!state_client->wait_for_service(timeout)) {
    RCLCPP_ERROR(get_logger(), "等待 /bt_navigator/get_state 服务超时");
    return false;
  }

  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
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
    // 使用辅助节点供 BT 叶节点使用
  auto node_ptr = bt_node_;

    // NavigateToStation：注入 Node 和 Nav2 Action Client
  factory_.registerBuilder<NavigateToStation>(
        "NavigateToStation",
    [node_ptr, this](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<NavigateToStation>(
                name, config, node_ptr, nav_client_);
        });

    // DockAtStation：注入 Node 和 cmd_vel Publisher
  factory_.registerBuilder<DockAtStation>(
        "DockAtStation",
    [node_ptr, this](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<DockAtStation>(
                name, config, node_ptr, cmd_vel_pub_);
        });

    // WaitForConfirmation：注入 Node 和确认标志指针
  factory_.registerBuilder<WaitForConfirmation>(
        "WaitForConfirmation",
    [node_ptr, this](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<WaitForConfirmation>(
                name, config, node_ptr, &load_confirmed_, &unload_confirmed_);
        });

    // ReportDeliveryStatus：注入 Node 和 Status Publisher
  factory_.registerBuilder<ReportDeliveryStatus>(
        "ReportDeliveryStatus",
    [node_ptr, this](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<ReportDeliveryStatus>(
                name, config, node_ptr, status_pub_);
        });

    // CheckBattery：条件节点，从黑板读取电量
  factory_.registerNodeType<CheckBattery>("CheckBattery");
}

// ======================== Action Server 回调 ========================

rclcpp_action::GoalResponse DeliveryExecutor::handle_goal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const ExecuteDelivery::Goal> goal)
{
  const auto & order = goal->order;

  RCLCPP_INFO(get_logger(), "收到配送请求 [%s]: %s → %s",
                order.order_id.c_str(),
                order.pickup_station.c_str(),
                order.dropoff_station.c_str());

    // 验证站点存在
  if (stations_.find(order.pickup_station) == stations_.end() ||
    stations_.find(order.dropoff_station) == stations_.end())
  {
    RCLCPP_WARN(get_logger(), "拒绝配送请求 [%s]：站点不存在",
                    order.order_id.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

    // 单任务模式：在 handle_goal 就预留执行槽位，避免和 handle_accepted 之间出现竞态窗口
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
  RCLCPP_INFO(get_logger(), "收到取消配送请求");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void DeliveryExecutor::handle_accepted(
  const std::shared_ptr<GoalHandleExecuteDelivery> goal_handle)
{
  // 等待前一个执行线程完成（正常情况下不会触发，因为 handle_goal 拒绝并发）
  join_bt_thread();

  stop_requested_.store(false, std::memory_order_release);
  {
    std::lock_guard<std::mutex> lock(execution_mutex_);
    active_goal_handle_ = goal_handle;
  }

  // 在新线程中执行 BT，避免阻塞 executor 线程
  bt_execution_thread_ = std::thread([this, goal_handle]() {
        execute_bt(goal_handle);
      });
}

// ======================== BT 执行核心循环 ========================

void DeliveryExecutor::execute_bt(
  const std::shared_ptr<GoalHandleExecuteDelivery> goal_handle)
{
  executing_.store(true);

  const auto & order = goal_handle->get_goal()->order;
  const auto start_time = std::chrono::steady_clock::now();
  const auto finish_execution = [this, &goal_handle]() {
      executing_.store(false, std::memory_order_release);
      goal_inflight_.store(false, std::memory_order_release);
      clear_active_goal(goal_handle);
    };

  RCLCPP_INFO(get_logger(), "开始执行配送 [%s]", order.order_id.c_str());

  // 重置确认标志，确保每次新订单从干净状态开始
  load_confirmed_.store(false, std::memory_order_release);
  unload_confirmed_.store(false, std::memory_order_release);

  // 从 XML 文件创建行为树实例
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

  // 设置黑板变量
  tree.rootBlackboard()->set("order_id", order.order_id);
  tree.rootBlackboard()->set("pickup_station", order.pickup_station);
  tree.rootBlackboard()->set("dropoff_station", order.dropoff_station);
  tree.rootBlackboard()->set("stations", stations_);
  tree.rootBlackboard()->set("battery_level", battery_level_.load());

  // 发布初始状态
  {
    DeliveryStatus status_msg;
    status_msg.stamp = this->now();
    status_msg.order_id = order.order_id;
    status_msg.state = DeliveryStatus::STATE_GOING_TO_PICKUP;
    status_msg.current_station = order.pickup_station;
    status_msg.progress = 0.1f;
    status_pub_->publish(status_msg);
  }

  // BT tick 循环
  BT::NodeStatus bt_status = BT::NodeStatus::RUNNING;
  rclcpp::Rate rate(100);  // 100Hz tick

  while (rclcpp::ok() && bt_status == BT::NodeStatus::RUNNING) {
    if (stop_requested_.load(std::memory_order_acquire)) {
      RCLCPP_INFO(get_logger(), "停止当前配送执行 [%s]", order.order_id.c_str());
      tree.haltTree();
      finish_execution();
      return;
    }

    // 检查取消请求
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

    // tick 一次行为树
    bt_status = tree.tickOnce();

    if (stop_requested_.load(std::memory_order_acquire)) {
      RCLCPP_INFO(get_logger(), "停止当前配送执行 [%s]", order.order_id.c_str());
      tree.haltTree();
      finish_execution();
      return;
    }

    // 从黑板读取 ReportDeliveryStatus 写入的最新状态作为 feedback
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

    rate.sleep();
  }

  if (stop_requested_.load(std::memory_order_acquire) || !rclcpp::ok()) {
    tree.haltTree();
    finish_execution();
    return;
  }

  // 构造结果
  auto result = std::make_shared<ExecuteDelivery::Result>();
  const auto elapsed = std::chrono::steady_clock::now() - start_time;
  result->elapsed_time_sec =
    std::chrono::duration_cast<std::chrono::duration<double>>(elapsed).count();

  if (bt_status == BT::NodeStatus::SUCCESS) {
    result->success = true;

        // 配送完成后扣减电量（原子操作，线程安全）
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
    if (current_battery < 20.0) {
      result->error_msg = "电量不足 (" +
        std::to_string(static_cast<int>(current_battery)) + "%)，配送中止";
    } else {
      result->error_msg = "行为树执行失败";
    }
    RCLCPP_WARN(get_logger(), "配送失败 [%s]: %s",
      order.order_id.c_str(), result->error_msg.c_str());

        // 发布失败状态
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

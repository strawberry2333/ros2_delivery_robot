/**
 * @file delivery_manager.cpp
 * @brief 配送管理节点实现。
 *
 * 从 Ros2Learning/task_runner 重构而来，将"导航→抓取→导航→放置"流程
 * 改为"导航取货点→等待装货确认→导航送货点→等待卸货确认"的真实配送模式。
 *
 * 本文件包含 DeliveryManager 类的全部方法实现：
 * - 构造函数：参数声明与通信接口初始化
 * - run()：主配送循环
 * - execute_delivery()：单订单配送流程
 * - 环境准备：时钟、TF、Action Server 等待
 * - 导航：Nav2 Action 调用
 * - 服务回调：订单提交、取消、报告
 * - 状态管理：状态发布与转换
 */

#include "delivery_core/delivery_manager.hpp"

#include <tf2/LinearMath/Quaternion.h>            // 四元数运算（欧拉角→四元数转换）
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // tf2 与 geometry_msgs 之间的类型转换
#include <tf2_ros/buffer.h>                        // TF 缓冲区
#include <tf2_ros/transform_listener.h>            // TF 监听器

#include <chrono>

using namespace std::chrono_literals;  // 启用 5s, 200ms 等时间字面量

namespace delivery_core
{

/**
 * @brief 构造函数实现：声明参数、创建通信接口。
 *
 * 初始化顺序很重要：
 * 1. 先声明参数（后续创建通信接口时可能依赖参数值）
 * 2. 创建回调组（服务创建时需要指定回调组）
 * 3. 创建 Action Client、Publisher、Service Server
 */
DeliveryManager::DeliveryManager(const rclcpp::NodeOptions & options)
: Node("delivery_manager", options)
{
    // === 声明参数 ===
    // 坐标系配置：通常不需要修改，除非使用自定义命名空间
  map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
  base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
    // 站点配置路径由 launch 文件通过参数传入
  station_config_path_ = this->declare_parameter<std::string>("station_config", "");

    // 各阶段的超时配置，可通过 YAML 参数文件调整
  clock_wait_timeout_sec_ = this->declare_parameter<double>(
        "clock_wait_timeout_sec", 30.0);
  tf_wait_timeout_sec_ = this->declare_parameter<double>(
        "tf_wait_timeout_sec", 15.0);
  navigation_timeout_sec_ = this->declare_parameter<double>(
        "navigation_timeout_sec", 120.0);
  wait_confirmation_timeout_sec_ = this->declare_parameter<double>(
        "wait_confirmation_timeout_sec", 60.0);
  action_server_wait_timeout_sec_ = this->declare_parameter<double>(
        "action_server_wait_timeout_sec", 60.0);
  cancel_completion_wait_timeout_sec_ = this->declare_parameter<double>(
        "cancel_completion_wait_timeout_sec", 10.0);

    // 初始位姿参数：用于 AMCL 定位初始化
  initial_x_ = this->declare_parameter<double>("initial_x", 0.0);
  initial_y_ = this->declare_parameter<double>("initial_y", 0.0);
  initial_yaw_ = this->declare_parameter<double>("initial_yaw", 0.0);
  publish_initial_pose_ = this->declare_parameter<bool>("publish_initial_pose", true);

    // use_sim_time 可能已由 launch 文件预设，需要先检查是否存在
    // 避免重复声明导致异常
  if (this->has_parameter("use_sim_time")) {
    this->get_parameter("use_sim_time", use_sim_time_);
  } else {
    use_sim_time_ = this->declare_parameter<bool>("use_sim_time", true);
  }

    // === 创建回调组 ===
    // 使用 Reentrant 类型，允许服务回调并发执行，
    // 避免与主线程的阻塞操作产生死锁
  service_cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::Reentrant);

    // === 创建通信接口 ===
    // 配送状态发布器（QoS depth=10，保留最近 10 条消息）
  status_pub_ = this->create_publisher<DeliveryStatus>("delivery_status", 10);
    // 初始位姿发布器，发布到全局话题 /initialpose（AMCL 订阅此话题）
  initial_pose_pub_ = this->create_publisher<PoseWithCovarianceStamped>(
        "/initialpose", 10);

    // === 创建服务端 ===
    // 所有服务都绑定到 service_cb_group_，确保不会被主线程阻塞
  submit_order_srv_ = this->create_service<delivery_interfaces::srv::SubmitOrder>(
        "submit_order",
        std::bind(&DeliveryManager::handle_submit_order, this,
                  std::placeholders::_1, std::placeholders::_2),
        rclcpp::ServicesQoS(), service_cb_group_);

  cancel_order_srv_ = this->create_service<delivery_interfaces::srv::CancelOrder>(
        "cancel_order",
        std::bind(&DeliveryManager::handle_cancel_order, this,
                  std::placeholders::_1, std::placeholders::_2),
        rclcpp::ServicesQoS(), service_cb_group_);

  get_report_srv_ = this->create_service<delivery_interfaces::srv::GetDeliveryReport>(
        "get_delivery_report",
        std::bind(&DeliveryManager::handle_get_report, this,
                  std::placeholders::_1, std::placeholders::_2),
        rclcpp::ServicesQoS(), service_cb_group_);

    // === 创建 ExecuteDelivery Action Client ===
    // 配送执行委托给 delivery_executor 节点
  delivery_action_client_ = rclcpp_action::create_client<ExecuteDelivery>(
        this, "execute_delivery");

    // 如果站点配置路径在构造时就可用，立即加载（便于测试和提前验证）
  if (!station_config_path_.empty()) {
    load_station_config(station_config_path_);
  }

  RCLCPP_INFO(get_logger(), "DeliveryManager 节点已初始化");
}

/**
 * @brief 主运行逻辑实现。
 *
 * 采用"准备→循环"两阶段模式：
 * - 准备阶段严格按顺序执行，任一步骤失败则直接返回（无法配送）
 * - 主循环以 2Hz 轮询订单队列，这个频率在响应速度和 CPU 占用之间取得平衡
 */
void DeliveryManager::run()
{
    // --- 准备阶段 1: 仿真时钟 ---
    // 在 use_sim_time 模式下必须先等待 /clock，否则 this->now() 始终返回 0
  if (use_sim_time_) {
    RCLCPP_INFO(get_logger(), "等待仿真时钟 /clock...");
    if (!wait_for_time()) {
      RCLCPP_ERROR(get_logger(), "等待时钟超时");
      return;
    }
  }

    // --- 准备阶段 2: 加载站点配置（构造函数可能已加载，此处仅在未加载时重试） ---
  if (stations_.empty() && !load_station_config(station_config_path_)) {
    RCLCPP_ERROR(get_logger(), "站点配置加载失败");
    return;
  }

    // --- 准备阶段 3: 发布初始位姿 ---
    // Nav2 的全局代价地图和 planner_server 需要 map→base_link TF，
    // 而该 TF 依赖 AMCL 收到初始位姿后才会建立。这里必须先发布初始位姿，
    // 避免与 executor/Nav2 的就绪等待形成启动死锁。
  if (publish_initial_pose_) {
    publish_initial_pose();
  }

    // --- 准备阶段 4: 等待 delivery_executor Action Server ---
    // executor 的 on_activate 会等待 bt_navigator 真正进入 Active。
  if (!wait_for_executor_server()) {
    RCLCPP_ERROR(get_logger(), "ExecuteDelivery action server 不可用");
    return;
  }

    // --- 准备阶段 5: 等待 TF 链路就绪 ---
    // map→base_link 需要 AMCL 收到初始位姿后才会出现
  if (!wait_for_tf()) {
    RCLCPP_ERROR(get_logger(), "TF 链路不可用");
    return;
  }

  system_ready_.store(true);
  RCLCPP_INFO(get_logger(),
        "系统就绪，已加载 %zu 个站点。等待订单提交...", stations_.size());

    // --- 主循环：从队列取订单并执行 ---
    // 使用 2Hz 轮询而非条件变量，原因是实现简单且对配送场景而言延迟可接受
  rclcpp::Rate rate(2.0);
  while (rclcpp::ok()) {
    OrderRecord current_order;
    bool has_order = false;

        // 使用最小化的锁范围：仅在访问共享队列时加锁，
        // 避免在执行配送（可能耗时数分钟）期间持有锁
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      if (!order_queue_.empty()) {
                // 取出队头订单（优先级最高的），拷贝后释放锁
        current_order = order_queue_.front();
        order_queue_.pop_front();
        has_order = true;
      }
    }

    if (has_order) {
            // 记录开始时间，用于后续统计配送耗时
      current_order.start_time = this->now();
      current_order.state = DeliveryState::kGoingToPickup;

            // 设置当前执行中的订单追踪信息
      {
        std::lock_guard<std::mutex> lock(current_order_mutex_);
        current_order_id_ = current_order.order.order_id;
        current_order_ = current_order;
      }

      RCLCPP_INFO(get_logger(), "开始执行订单 [%s]: %s → %s",
                        current_order.order.order_id.c_str(),
                        current_order.order.pickup_station.c_str(),
                        current_order.order.dropoff_station.c_str());

            // 执行配送全流程（阻塞，可能耗时数分钟）
      bool success = execute_delivery(current_order);

            // 清除当前订单追踪
      {
        std::lock_guard<std::mutex> lock(current_order_mutex_);
        current_order_id_.clear();
        current_order_.reset();
      }

            // 将完成的订单存入历史记录（无论成功还是失败）
      {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        completed_orders_.push_back(current_order);
      }

      if (success) {
        RCLCPP_INFO(get_logger(), "订单 [%s] 配送完成",
                            current_order.order.order_id.c_str());
      } else {
        RCLCPP_WARN(get_logger(), "订单 [%s] 配送失败: %s",
                            current_order.order.order_id.c_str(),
                            current_order.error_msg.c_str());
      }
    }

        // 没有订单时 sleep，减少 CPU 空转
    rate.sleep();
  }
}

/**
 * @brief 执行单个配送订单——通过 Action Client 调用 delivery_executor。
 *
 * 通过 Action Client 将配送流程委托给 delivery_executor 的行为树执行。
 */
bool DeliveryManager::execute_delivery(OrderRecord & record)
{
  const auto & order = record.order;

    // 构造 ExecuteDelivery Goal
  auto goal = ExecuteDelivery::Goal();
  goal.order = order;

  record.state = DeliveryState::kGoingToPickup;
  publish_status(order.order_id, DeliveryState::kGoingToPickup,
                   order.pickup_station, 0.1f);

    // 配置回调
  auto send_goal_options =
    rclcpp_action::Client<ExecuteDelivery>::SendGoalOptions();

    // Feedback 回调：将 executor 的实时状态同步到 current_order_，
    // 使 GetDeliveryReport 能返回准确的在途订单状态
  send_goal_options.feedback_callback =
    [this, &record](ExecuteDeliveryGoalHandle::SharedPtr,
    const std::shared_ptr<const ExecuteDelivery::Feedback> feedback)
    {
        // 将 msg 常量映射回内部枚举
      DeliveryState new_state = DeliveryState::kIdle;
      switch (feedback->state) {
        case DeliveryStatus::STATE_GOING_TO_PICKUP:  new_state = DeliveryState::kGoingToPickup;
          break;
        case DeliveryStatus::STATE_WAITING_LOAD:     new_state = DeliveryState::kWaitingLoad; break;
        case DeliveryStatus::STATE_GOING_TO_DROPOFF: new_state = DeliveryState::kGoingToDropoff;
          break;
        case DeliveryStatus::STATE_WAITING_UNLOAD:   new_state = DeliveryState::kWaitingUnload;
          break;
        case DeliveryStatus::STATE_COMPLETE:         new_state = DeliveryState::kComplete; break;
        case DeliveryStatus::STATE_FAILED:           new_state = DeliveryState::kFailed; break;
        default: return;
      }

      record.state = new_state;
      record.current_station = feedback->current_station;
      record.progress = feedback->progress;
      std::lock_guard<std::mutex> lock(current_order_mutex_);
      if (current_order_.has_value()) {
        current_order_->state = new_state;
        current_order_->current_station = feedback->current_station;
        current_order_->progress = feedback->progress;
      }
    };

    // 发送 Goal
  RCLCPP_INFO(get_logger(), "向 executor 发送配送请求 [%s]",
                order.order_id.c_str());

  auto goal_future =
    delivery_action_client_->async_send_goal(goal, send_goal_options);

  if (goal_future.wait_for(10s) != std::future_status::ready) {
    record.state = DeliveryState::kFailed;
    record.error_msg = "发送配送目标超时";
    publish_status(order.order_id, DeliveryState::kFailed, "", 0.0f,
                       record.error_msg);
    return false;
  }

  auto goal_handle = goal_future.get();
  if (!goal_handle) {
    record.state = DeliveryState::kFailed;
    record.error_msg = "配送目标被 executor 拒绝";
    publish_status(order.order_id, DeliveryState::kFailed, "", 0.0f,
                       record.error_msg);
    return false;
  }

    // 保存 goal handle 以便取消
  {
    std::lock_guard<std::mutex> lock(goal_handle_mutex_);
    current_goal_handle_ = goal_handle;
  }

    // 等待结果——总超时 = 2 * 导航超时 + 2 * 确认超时 + 余量
  const double total_timeout =
    2.0 * navigation_timeout_sec_ + 2.0 * wait_confirmation_timeout_sec_ + 60.0;
  const std::chrono::duration<double> timeout(total_timeout);

  auto result_future = delivery_action_client_->async_get_result(goal_handle);

  if (result_future.wait_for(timeout) != std::future_status::ready) {
    record.state = DeliveryState::kFailed;
    record.error_msg = "配送执行总超时";

        // 超时后取消 Goal，并等待 executor 真正进入终态，避免下一单立刻被旧任务占住
    delivery_action_client_->async_cancel_goal(goal_handle);
    (void)wait_for_terminal_result_after_cancel(order.order_id, result_future);

    publish_status(order.order_id, DeliveryState::kFailed, "", 0.0f,
                       record.error_msg);

    {
      std::lock_guard<std::mutex> lock(goal_handle_mutex_);
      current_goal_handle_.reset();
    }
    return false;
  }

    // 清除 goal handle
  {
    std::lock_guard<std::mutex> lock(goal_handle_mutex_);
    current_goal_handle_.reset();
  }

  const auto wrapped_result = result_future.get();

  if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED &&
    wrapped_result.result->success)
  {
    record.state = DeliveryState::kComplete;
    RCLCPP_INFO(get_logger(), "订单 [%s] 由 executor 完成，耗时 %.1f 秒",
                    order.order_id.c_str(),
                    wrapped_result.result->elapsed_time_sec);
    return true;
  }

    // 被取消
  if (wrapped_result.code == rclcpp_action::ResultCode::CANCELED) {
    record.state = DeliveryState::kCanceled;
    record.error_msg = "配送被用户取消";
    publish_status(order.order_id, DeliveryState::kCanceled, "", 0.0f,
                       record.error_msg);
    return false;
  }

    // 失败
  record.state = DeliveryState::kFailed;
  record.error_msg = wrapped_result.result ?
    wrapped_result.result->error_msg : "executor 返回失败";
  publish_status(order.order_id, DeliveryState::kFailed, "", 0.0f,
                   record.error_msg);
  return false;
}

/**
 * @brief 等待 /execute_delivery Action Server 可用。
 */
bool DeliveryManager::wait_for_executor_server()
{
  RCLCPP_INFO(get_logger(), "等待 ExecuteDelivery action server...");
  const std::chrono::duration<double> timeout(action_server_wait_timeout_sec_);
  return delivery_action_client_->wait_for_action_server(
        std::chrono::duration_cast<std::chrono::milliseconds>(timeout));
}

bool DeliveryManager::wait_for_terminal_result_after_cancel(
  const std::string & order_id,
  const std::shared_future<ExecuteDeliveryGoalHandle::WrappedResult> & result_future)
{
  const std::chrono::duration<double> timeout(cancel_completion_wait_timeout_sec_);
  if (result_future.wait_for(timeout) == std::future_status::ready) {
    return true;
  }

  RCLCPP_WARN(get_logger(),
        "订单 [%s] 的取消请求已发送，但 %.1f 秒内未等到 executor 进入终态",
        order_id.c_str(), cancel_completion_wait_timeout_sec_);
  return false;
}

// ======================== 服务回调实现 ========================

/**
 * @brief 订单提交服务回调。
 *
 * 多层验证确保只有合法订单才能进入队列：
 * 1. order_id 非空检查
 * 2. 站点存在性检查
 * 3. 站点类型匹配检查（取货站不能是 dropoff 类型，反之亦然）
 * 4. order_id 唯一性检查（队列和历史记录中均不能重复）
 *
 * 优先级排序采用插入排序方式，保证队列始终有序。
 */
void DeliveryManager::handle_submit_order(
  const std::shared_ptr<delivery_interfaces::srv::SubmitOrder::Request> request,
  std::shared_ptr<delivery_interfaces::srv::SubmitOrder::Response> response)
{
  const auto & order = request->order;

    // 系统启动检查完成前拒绝订单，避免 run() 退出后订单被静默丢弃
  if (!system_ready_.load()) {
    response->accepted = false;
    response->reason = "系统尚未就绪，请稍后重试";
    return;
  }

    // 验证 order_id 非空——空 ID 无法用于状态追踪和报告查询
  if (order.order_id.empty()) {
    response->accepted = false;
    response->reason = "order_id 不能为空";
    return;
  }

    // 验证取货站点存在
  auto pickup_it = stations_.find(order.pickup_station);
  if (pickup_it == stations_.end()) {
    response->accepted = false;
    response->reason = "取货站点不存在: " + order.pickup_station;
    return;
  }
    // 验证送货站点存在
  auto dropoff_it = stations_.find(order.dropoff_station);
  if (dropoff_it == stations_.end()) {
    response->accepted = false;
    response->reason = "送货站点不存在: " + order.dropoff_station;
    return;
  }

    // 取货站和送货站不能相同
  if (order.pickup_station == order.dropoff_station) {
    response->accepted = false;
    response->reason = "取货站和送货站不能相同: " + order.pickup_station;
    return;
  }

    // 白名单验证站点类型：
    // type=0 是取货站，type=1 是送货站，type=2 是充电桩
    // 取货点必须是 type==0，送货点必须是 type==1，充电桩不可作为业务站点
  if (pickup_it->second.type != 0) {
    response->accepted = false;
    response->reason = "站点 " + order.pickup_station +
      " (type=" + std::to_string(pickup_it->second.type) + ") 不是取货站，不能作为取货点";
    return;
  }
  if (dropoff_it->second.type != 1) {
    response->accepted = false;
    response->reason = "站点 " + order.dropoff_station +
      " (type=" + std::to_string(dropoff_it->second.type) + ") 不是送货站，不能作为送货点";
    return;
  }

    // 构造订单记录
  OrderRecord record;
  record.order = order;
  record.state = DeliveryState::kIdle;

  {
    std::lock_guard<std::mutex> lock(queue_mutex_);

        // 检查 order_id 在待执行队列中是否重复
    for (const auto & existing : order_queue_) {
      if (existing.order.order_id == order.order_id) {
        response->accepted = false;
        response->reason = "订单 ID 重复: " + order.order_id;
        return;
      }
    }
        // 检查 order_id 是否与当前执行中的订单重复
    {
      std::lock_guard<std::mutex> co_lock(current_order_mutex_);
      if (!current_order_id_.empty() && current_order_id_ == order.order_id) {
        response->accepted = false;
        response->reason = "订单 ID 正在执行中: " + order.order_id;
        return;
      }
    }
        // 检查 order_id 在历史记录中是否重复
        // 防止同一订单被重复提交
    for (const auto & existing : completed_orders_) {
      if (existing.order.order_id == order.order_id) {
        response->accepted = false;
        response->reason = "订单 ID 已存在于历史记录: " + order.order_id;
        return;
      }
    }

        // 按优先级插入：遍历队列找到第一个优先级低于当前订单的位置
        // priority 值越大优先级越高，高优先级排在队列前端
    auto it = order_queue_.begin();
    while (it != order_queue_.end() && it->order.priority >= order.priority) {
      ++it;
    }
    order_queue_.insert(it, record);
  }

  response->accepted = true;
  response->reason = "订单已加入队列";

  RCLCPP_INFO(get_logger(), "接收订单 [%s]: %s → %s (优先级: %u)",
                order.order_id.c_str(),
                order.pickup_station.c_str(),
                order.dropoff_station.c_str(),
                order.priority);
}

/**
 * @brief 订单取消服务回调。
 *
 * 仅能取消队列中尚未开始执行的订单。
 * 当前已支持取消执行中订单（通过 ExecuteDelivery Action 取消）。
 */
void DeliveryManager::handle_cancel_order(
  const std::shared_ptr<delivery_interfaces::srv::CancelOrder::Request> request,
  std::shared_ptr<delivery_interfaces::srv::CancelOrder::Response> response)
{
  std::lock_guard<std::mutex> lock(queue_mutex_);

    // 在队列中查找目标订单
  auto it = std::find_if(order_queue_.begin(), order_queue_.end(),
      [&request](const OrderRecord & r) {
        return r.order.order_id == request->order_id;
        });

  if (it != order_queue_.end()) {
    order_queue_.erase(it);
    response->success = true;
    response->reason = "订单已从队列中移除";
    RCLCPP_INFO(get_logger(), "取消订单 [%s]", request->order_id.c_str());
  } else {
        // 尝试取消正在执行中的订单——需要验证 order_id 匹配
    std::lock_guard<std::mutex> co_lock(current_order_mutex_);
    std::lock_guard<std::mutex> gh_lock(goal_handle_mutex_);
    if (current_goal_handle_ && current_order_id_ == request->order_id) {
      RCLCPP_INFO(get_logger(), "取消执行中的订单 [%s]",
                        request->order_id.c_str());
      delivery_action_client_->async_cancel_goal(current_goal_handle_);
      response->success = true;
      response->reason = "已发送取消请求到 executor";
    } else {
      response->success = false;
      response->reason = "订单不在队列中且不是当前执行中的订单（可能已完成或从未提交）";
    }
  }
}

/**
 * @brief 配送报告查询服务回调。
 *
 * 汇总所有订单状态：包括排队中的和已完成/失败的。
 * 汇总排队中、执行中和已完成/失败的所有订单状态。
 */
void DeliveryManager::handle_get_report(
  const std::shared_ptr<delivery_interfaces::srv::GetDeliveryReport::Request>,
  std::shared_ptr<delivery_interfaces::srv::GetDeliveryReport::Response> response)
{
  std::lock_guard<std::mutex> lock(queue_mutex_);

    // 排队中的订单
  for (const auto & record : order_queue_) {
    DeliveryStatus status;
    status.order_id = record.order.order_id;
    status.state = state_to_msg(record.state);
    status.error_msg = record.error_msg;
    response->reports.push_back(status);
  }

    // 当前正在执行的订单
  {
    std::lock_guard<std::mutex> co_lock(current_order_mutex_);
    if (current_order_.has_value()) {
      DeliveryStatus status;
      status.order_id = current_order_->order.order_id;
      status.state = state_to_msg(current_order_->state);
      status.current_station = current_order_->current_station;
      status.progress = current_order_->progress;
      status.error_msg = current_order_->error_msg;
      response->reports.push_back(status);
    }
  }

    // 已完成/失败的订单
  for (const auto & record : completed_orders_) {
    DeliveryStatus status;
    status.order_id = record.order.order_id;
    status.state = state_to_msg(record.state);
    status.error_msg = record.error_msg;
    response->reports.push_back(status);
  }
}

// ======================== 配置加载 ========================

/**
 * @brief 从 YAML 文件加载站点配置的实现。
 *
 * YAML 格式要求：
 *   stations:
 *     - station_id: "station_A"
 *       x: 1.0
 *       y: 2.0
 *       yaw: 0.0           # 可选，默认 0.0
 *       station_type: 0    # 可选，默认 0 (pickup)
 *
 * 加载过程中会进行严格校验：
 * - station_type 必须在 [0, 1, 2] 范围内
 * - station_id 不能重复
 * - 至少需要一个站点
 */
bool DeliveryManager::load_station_config(const std::string & path)
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

    // 检查 YAML 根节点中是否包含 "stations" 键，且其值为列表类型
  const auto stations_node = root["stations"];
  if (!stations_node || !stations_node.IsSequence()) {
    RCLCPP_ERROR(get_logger(), "YAML 中缺少 'stations' 列表");
    return false;
  }

    // 清空已有数据，支持重新加载（虽然当前版本只在启动时加载一次）
  stations_.clear();
  for (size_t i = 0; i < stations_node.size(); ++i) {
    const auto & node = stations_node[i];
    Station station;
    station.id = node["station_id"].as<std::string>();
    station.pose.x = node["x"].as<double>();
    station.pose.y = node["y"].as<double>();
        // yaw 和 station_type 是可选字段，提供合理的默认值
    station.pose.yaw = node["yaw"] ? node["yaw"].as<double>() : 0.0;
    station.type = node["station_type"] ? node["station_type"].as<uint8_t>() : 0;

        // 校验 station_type 合法性 (0=pickup, 1=dropoff, 2=charge)
    if (station.type > 2) {
      RCLCPP_ERROR(get_logger(), "站点 [%s] 类型非法: %u (应为 0/1/2)",
                         station.id.c_str(), station.type);
      stations_.clear();
      return false;
    }

        // 校验 station_id 不重复——重复 ID 会导致后续查找歧义
    if (stations_.count(station.id) > 0) {
      RCLCPP_ERROR(get_logger(), "站点 ID 重复: %s", station.id.c_str());
      stations_.clear();
      return false;
    }

    stations_[station.id] = station;
    RCLCPP_INFO(get_logger(), "  站点 [%s]: (%.2f, %.2f, %.2f) 类型=%u",
                    station.id.c_str(), station.pose.x, station.pose.y,
                    station.pose.yaw, station.type);
  }

    // 至少需要一个站点才算加载成功
  return !stations_.empty();
}

// ======================== 环境准备（复用 Ros2Learning/task_runner 模式） ========================

/**
 * @brief 等待仿真时钟实现。
 *
 * 复用 Ros2Learning 中 task_runner 的模式：
 * Gazebo 启动后会发布 /clock 话题，ROS 节点的时钟才会从 0 变为实际仿真时间。
 * 100ms 轮询间隔足够快速响应，不会显著增加 CPU 负载。
 */
bool DeliveryManager::wait_for_time()
{
    // 使用挂钟时间（steady_clock）计算超时，因为此时 ROS 时钟尚不可用
  const auto start_wall = std::chrono::steady_clock::now();
  while (rclcpp::ok()) {
        // 时钟值非零说明已收到 /clock 消息
    if (this->get_clock()->now().nanoseconds() != 0) {
      return true;
    }

    const auto elapsed = std::chrono::steady_clock::now() - start_wall;
    if (std::chrono::duration_cast<std::chrono::duration<double>>(elapsed)
      .count() > clock_wait_timeout_sec_)
    {
      return false;
    }
    rclcpp::sleep_for(100ms);
  }
  return false;
}

/**
 * @brief 等待 TF 链路就绪的实现。
 *
 * 创建局部 TF Buffer 和 Listener 来查询 map→base_link 变换。
 * 使用 tf2::TimePointZero 查询最新可用的变换（而非指定时间点），
 * 因为此时只需要确认链路是否存在，不关心变换的时间戳。
 *
 * @note TF Buffer 和 Listener 是局部变量，方法返回后自动销毁。
 *       运行期间的 TF 查询由 Nav2 内部处理，不需要本节点维护。
 */
bool DeliveryManager::wait_for_tf()
{
  tf2_ros::Buffer tf_buffer(this->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer);

  RCLCPP_INFO(get_logger(), "等待 TF 链路: [%s] → [%s]...",
                map_frame_.c_str(), base_frame_.c_str());

  const auto start_time = this->now();
  while (rclcpp::ok()) {
    try {
            // 尝试查询 map→base_link 的变换
      tf_buffer.lookupTransform(map_frame_, base_frame_,
                                      tf2::TimePointZero);
      RCLCPP_INFO(get_logger(), "TF 链路就绪");
      return true;
    } catch (const tf2::TransformException &) {
            // 变换尚不可用，等待后重试
      rclcpp::sleep_for(200ms);
    }

        // 此处使用 ROS 时钟计算超时（此时 ROS 时钟已可用）
    if ((this->now() - start_time).seconds() > tf_wait_timeout_sec_) {
      return false;
    }
  }
  return false;
}


/**
 * @brief 发布初始位姿的实现。
 *
 * 关键细节：
 * - 协方差矩阵只设置了对角线上的 x(索引0)、y(索引7)、yaw(索引35) 三个值
 *   其他元素保持为 0，表示各维度之间无相关性
 * - 0.25 的协方差值对应 0.5 米/弧度的标准差，这是比较宽松的初始估计
 * - 连续发布 5 次是经验做法：AMCL 启动瞬间可能丢失订阅，多次发布提高可靠性
 */
void DeliveryManager::publish_initial_pose()
{
  PoseWithCovarianceStamped msg;
  msg.header.frame_id = map_frame_;
  msg.header.stamp = this->now();
  msg.pose.pose.position.x = initial_x_;
  msg.pose.pose.position.y = initial_y_;

    // 将欧拉角（yaw）转换为四元数，roll 和 pitch 设为 0（平面运动）
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, initial_yaw_);
  msg.pose.pose.orientation = tf2::toMsg(q);

    // 设置协方差矩阵的对角线元素（6x6 矩阵，按行主序展开为 36 元素数组）
    // 索引 0: x 方向的方差, 索引 7: y 方向的方差, 索引 35: yaw 方向的方差
  msg.pose.covariance[0] = 0.25;
  msg.pose.covariance[7] = 0.25;
  msg.pose.covariance[35] = 0.25;

  RCLCPP_INFO(get_logger(), "发布初始位姿到 /initialpose");
    // 连续发布 5 次，间隔 200ms，确保 AMCL 接收到
  for (int i = 0; rclcpp::ok() && i < 5; ++i) {
        // 每次更新时间戳，确保消息不会因时间戳过旧被丢弃
    msg.header.stamp = this->now();
    initial_pose_pub_->publish(msg);
    rclcpp::sleep_for(200ms);
  }
}

// ======================== 状态管理 ========================

/**
 * @brief 发布配送状态的实现。
 *
 * 每次状态变更时调用，构造 DeliveryStatus 消息并发布。
 * 同时打印日志，便于调试时在终端直接观察状态变化。
 */
void DeliveryManager::publish_status(
  const std::string & order_id, DeliveryState state,
  const std::string & station, float progress, const std::string & error)
{
  DeliveryStatus msg;
  msg.stamp = this->now();
  msg.order_id = order_id;
  msg.state = state_to_msg(state);       // 枚举转为消息常量
  msg.current_station = station;
  msg.progress = progress;
  msg.error_msg = error;
  status_pub_->publish(msg);

    // 日志输出：进度以百分比显示，更直观
  RCLCPP_INFO(get_logger(), "状态更新 [%s]: %s (站点: %s, 进度: %.0f%%)",
                order_id.c_str(), state_to_string(state).c_str(),
                station.c_str(), progress * 100.0f);
}

/**
 * @brief 内部状态枚举到消息常量的映射。
 *
 * DeliveryStatus.msg 中通过 uint8 常量定义了所有可能的状态值，
 * 本方法确保内部枚举与消息定义保持一致。
 * default 分支返回 STATE_IDLE 作为安全兜底。
 */
uint8_t DeliveryManager::state_to_msg(DeliveryState state) const
{
  switch (state) {
    case DeliveryState::kIdle:           return DeliveryStatus::STATE_IDLE;
    case DeliveryState::kGoingToPickup:  return DeliveryStatus::STATE_GOING_TO_PICKUP;
    case DeliveryState::kWaitingLoad:    return DeliveryStatus::STATE_WAITING_LOAD;
    case DeliveryState::kGoingToDropoff: return DeliveryStatus::STATE_GOING_TO_DROPOFF;
    case DeliveryState::kWaitingUnload:  return DeliveryStatus::STATE_WAITING_UNLOAD;
    case DeliveryState::kComplete:       return DeliveryStatus::STATE_COMPLETE;
    case DeliveryState::kFailed:         return DeliveryStatus::STATE_FAILED;
    case DeliveryState::kCanceled:       return DeliveryStatus::STATE_CANCELED;
    default:                             return DeliveryStatus::STATE_IDLE;
  }
}

/**
 * @brief 状态枚举到可读字符串的映射，用于日志输出。
 *
 * 格式为"中文(English)"，方便中英文环境下都能快速识别状态。
 */
std::string DeliveryManager::state_to_string(DeliveryState state) const
{
  switch (state) {
    case DeliveryState::kIdle:           return "空闲(Idle)";
    case DeliveryState::kGoingToPickup:  return "前往取货点(GoingToPickup)";
    case DeliveryState::kWaitingLoad:    return "等待装货(WaitingLoad)";
    case DeliveryState::kGoingToDropoff: return "前往送货点(GoingToDropoff)";
    case DeliveryState::kWaitingUnload:  return "等待卸货(WaitingUnload)";
    case DeliveryState::kComplete:       return "配送完成(Complete)";
    case DeliveryState::kFailed:         return "配送失败(Failed)";
    case DeliveryState::kCanceled:       return "配送取消(Canceled)";
    default:                             return "未知(Unknown)";
  }
}

}  // namespace delivery_core

/**
 * @file delivery_manager.cpp
 * @brief 配送管理节点实现。
 *
 * 从 Ros2Learning/task_runner 重构而来，将"导航→抓取→导航→放置"流程
 * 改为"导航取货点→等待装货确认→导航送货点→等待卸货确认"的真实配送模式。
 */

#include "delivery_core/delivery_manager.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>

using namespace std::chrono_literals;

namespace delivery_core
{

DeliveryManager::DeliveryManager()
    : Node("delivery_manager")
{
    // --- 声明参数 ---
    map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
    nav2_action_name_ = this->declare_parameter<std::string>(
        "nav2_action_name", "navigate_to_pose");
    station_config_path_ = this->declare_parameter<std::string>("station_config", "");

    clock_wait_timeout_sec_ = this->declare_parameter<double>(
        "clock_wait_timeout_sec", 30.0);
    tf_wait_timeout_sec_ = this->declare_parameter<double>(
        "tf_wait_timeout_sec", 15.0);
    navigation_timeout_sec_ = this->declare_parameter<double>(
        "navigation_timeout_sec", 120.0);
    wait_confirmation_timeout_sec_ = this->declare_parameter<double>(
        "wait_confirmation_timeout_sec", 60.0);

    initial_x_ = this->declare_parameter<double>("initial_x", 0.0);
    initial_y_ = this->declare_parameter<double>("initial_y", 0.0);
    initial_yaw_ = this->declare_parameter<double>("initial_yaw", 0.0);
    publish_initial_pose_ = this->declare_parameter<bool>("publish_initial_pose", true);

    if (this->has_parameter("use_sim_time"))
    {
        this->get_parameter("use_sim_time", use_sim_time_);
    }
    else
    {
        use_sim_time_ = this->declare_parameter<bool>("use_sim_time", true);
    }

    // --- 创建回调组 ---
    service_cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::Reentrant);

    // --- 创建通信接口 ---
    action_client_ = rclcpp_action::create_client<NavigateToPose>(
        this, nav2_action_name_);

    status_pub_ = this->create_publisher<DeliveryStatus>("delivery_status", 10);
    initial_pose_pub_ = this->create_publisher<PoseWithCovarianceStamped>(
        "/initialpose", 10);

    // --- 创建服务端 ---
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

    // --- 创建装/卸货确认服务端 ---
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

    RCLCPP_INFO(get_logger(), "DeliveryManager 节点已初始化");
}

void DeliveryManager::run()
{
    // --- 准备阶段 1: 仿真时钟 ---
    if (use_sim_time_)
    {
        RCLCPP_INFO(get_logger(), "等待仿真时钟 /clock...");
        if (!wait_for_time())
        {
            RCLCPP_ERROR(get_logger(), "等待时钟超时");
            return;
        }
    }

    // --- 准备阶段 2: 加载站点配置 ---
    if (!load_station_config(station_config_path_))
    {
        RCLCPP_ERROR(get_logger(), "站点配置加载失败");
        return;
    }

    // --- 准备阶段 3: 等待导航服务 ---
    if (!wait_for_action_server())
    {
        RCLCPP_ERROR(get_logger(), "Nav2 action server 不可用");
        return;
    }

    // 发布初始位姿
    if (publish_initial_pose_)
    {
        publish_initial_pose();
    }

    // 等待 TF
    if (!wait_for_tf())
    {
        RCLCPP_ERROR(get_logger(), "TF 链路不可用");
        return;
    }

    RCLCPP_INFO(get_logger(),
        "系统就绪，已加载 %zu 个站点。等待订单提交...", stations_.size());

    // --- 主循环：从队列取订单并执行 ---
    rclcpp::Rate rate(2.0);  // 2Hz 轮询
    while (rclcpp::ok())
    {
        OrderRecord current_order;
        bool has_order = false;

        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            if (!order_queue_.empty())
            {
                current_order = order_queue_.front();
                order_queue_.pop_front();
                has_order = true;
            }
        }

        if (has_order)
        {
            current_order.start_time = this->now();
            RCLCPP_INFO(get_logger(), "开始执行订单 [%s]: %s → %s",
                        current_order.order.order_id.c_str(),
                        current_order.order.pickup_station.c_str(),
                        current_order.order.dropoff_station.c_str());

            bool success = execute_delivery(current_order);

            {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                completed_orders_.push_back(current_order);
            }

            if (success)
            {
                RCLCPP_INFO(get_logger(), "订单 [%s] 配送完成",
                            current_order.order.order_id.c_str());
            }
            else
            {
                RCLCPP_WARN(get_logger(), "订单 [%s] 配送失败: %s",
                            current_order.order.order_id.c_str(),
                            current_order.error_msg.c_str());
            }
        }

        rate.sleep();
    }
}

bool DeliveryManager::execute_delivery(OrderRecord & record)
{
    const auto & order = record.order;

    // 查找站点
    auto pickup_it = stations_.find(order.pickup_station);
    auto dropoff_it = stations_.find(order.dropoff_station);

    if (pickup_it == stations_.end())
    {
        record.state = DeliveryState::kFailed;
        record.error_msg = "取货站点不存在: " + order.pickup_station;
        publish_status(order.order_id, DeliveryState::kFailed, "", 0.0f,
                       record.error_msg);
        return false;
    }
    if (dropoff_it == stations_.end())
    {
        record.state = DeliveryState::kFailed;
        record.error_msg = "送货站点不存在: " + order.dropoff_station;
        publish_status(order.order_id, DeliveryState::kFailed, "", 0.0f,
                       record.error_msg);
        return false;
    }

    const auto & pickup = pickup_it->second;
    const auto & dropoff = dropoff_it->second;

    // [阶段 1] 导航到取货点
    record.state = DeliveryState::kGoingToPickup;
    publish_status(order.order_id, DeliveryState::kGoingToPickup,
                   pickup.id, 0.1f);

    if (!navigate_to(pickup.pose, "取货点 " + pickup.id))
    {
        record.state = DeliveryState::kFailed;
        record.error_msg = "导航到取货点失败";
        publish_status(order.order_id, DeliveryState::kFailed,
                       pickup.id, 0.0f, record.error_msg);
        return false;
    }

    // [阶段 2] 等待装货确认
    record.state = DeliveryState::kWaitingLoad;
    publish_status(order.order_id, DeliveryState::kWaitingLoad,
                   pickup.id, 0.3f);

    if (!wait_for_confirmation("confirm_load", order.order_id,
                               pickup.id, DeliveryState::kWaitingLoad))
    {
        record.state = DeliveryState::kFailed;
        record.error_msg = "等待装货确认超时";
        publish_status(order.order_id, DeliveryState::kFailed,
                       pickup.id, 0.0f, record.error_msg);
        return false;
    }

    // [阶段 3] 导航到送货点
    record.state = DeliveryState::kGoingToDropoff;
    publish_status(order.order_id, DeliveryState::kGoingToDropoff,
                   dropoff.id, 0.5f);

    if (!navigate_to(dropoff.pose, "送货点 " + dropoff.id))
    {
        record.state = DeliveryState::kFailed;
        record.error_msg = "导航到送货点失败";
        publish_status(order.order_id, DeliveryState::kFailed,
                       dropoff.id, 0.0f, record.error_msg);
        return false;
    }

    // [阶段 4] 等待卸货确认
    record.state = DeliveryState::kWaitingUnload;
    publish_status(order.order_id, DeliveryState::kWaitingUnload,
                   dropoff.id, 0.8f);

    if (!wait_for_confirmation("confirm_unload", order.order_id,
                               dropoff.id, DeliveryState::kWaitingUnload))
    {
        record.state = DeliveryState::kFailed;
        record.error_msg = "等待卸货确认超时";
        publish_status(order.order_id, DeliveryState::kFailed,
                       dropoff.id, 0.0f, record.error_msg);
        return false;
    }

    // [完成]
    record.state = DeliveryState::kComplete;
    publish_status(order.order_id, DeliveryState::kComplete,
                   dropoff.id, 1.0f);
    return true;
}

bool DeliveryManager::wait_for_confirmation(
    const std::string & confirm_service,
    const std::string & order_id,
    const std::string & station_id,
    DeliveryState state)
{
    // 选择对应的确认标志
    std::atomic<bool> & confirmed =
        (state == DeliveryState::kWaitingLoad) ? load_confirmed_ : unload_confirmed_;

    // 重置确认标志
    confirmed.store(false, std::memory_order_release);

    const std::string action_label =
        (state == DeliveryState::kWaitingLoad) ? "装货" : "卸货";

    RCLCPP_INFO(get_logger(),
        "订单 [%s] 在站点 [%s] 等待%s确认... (调用 ros2 service call /%s std_srvs/srv/Trigger)",
        order_id.c_str(), station_id.c_str(), action_label.c_str(),
        confirm_service.c_str());

    const auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok())
    {
        if (confirmed.load(std::memory_order_acquire))
        {
            RCLCPP_INFO(get_logger(), "订单 [%s] %s确认完成",
                        order_id.c_str(), action_label.c_str());
            return true;
        }

        const auto elapsed = std::chrono::steady_clock::now() - start;
        if (std::chrono::duration_cast<std::chrono::duration<double>>(elapsed)
                .count() > wait_confirmation_timeout_sec_)
        {
            RCLCPP_WARN(get_logger(),
                "订单 [%s] %s确认超时 (%.0f 秒)",
                order_id.c_str(), action_label.c_str(),
                wait_confirmation_timeout_sec_);
            return false;  // 超时视为失败，与头文件注释语义一致
        }

        rclcpp::sleep_for(500ms);
    }
    return false;
}

// --- 服务回调实现 ---

void DeliveryManager::handle_submit_order(
    const std::shared_ptr<delivery_interfaces::srv::SubmitOrder::Request> request,
    std::shared_ptr<delivery_interfaces::srv::SubmitOrder::Response> response)
{
    const auto & order = request->order;

    // 验证站点存在
    if (stations_.find(order.pickup_station) == stations_.end())
    {
        response->accepted = false;
        response->reason = "取货站点不存在: " + order.pickup_station;
        return;
    }
    if (stations_.find(order.dropoff_station) == stations_.end())
    {
        response->accepted = false;
        response->reason = "送货站点不存在: " + order.dropoff_station;
        return;
    }

    // 加入队列
    OrderRecord record;
    record.order = order;
    record.state = DeliveryState::kIdle;

    {
        std::lock_guard<std::mutex> lock(queue_mutex_);

        // 按优先级插入（高优先级在前）
        auto it = order_queue_.begin();
        while (it != order_queue_.end() && it->order.priority >= order.priority)
        {
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

void DeliveryManager::handle_cancel_order(
    const std::shared_ptr<delivery_interfaces::srv::CancelOrder::Request> request,
    std::shared_ptr<delivery_interfaces::srv::CancelOrder::Response> response)
{
    std::lock_guard<std::mutex> lock(queue_mutex_);

    auto it = std::find_if(order_queue_.begin(), order_queue_.end(),
        [&request](const OrderRecord & r) {
            return r.order.order_id == request->order_id;
        });

    if (it != order_queue_.end())
    {
        order_queue_.erase(it);
        response->success = true;
        response->reason = "订单已从队列中移除";
        RCLCPP_INFO(get_logger(), "取消订单 [%s]", request->order_id.c_str());
    }
    else
    {
        response->success = false;
        response->reason = "订单不在队列中（可能已在执行或已完成）";
    }
}

void DeliveryManager::handle_get_report(
    const std::shared_ptr<delivery_interfaces::srv::GetDeliveryReport::Request>,
    std::shared_ptr<delivery_interfaces::srv::GetDeliveryReport::Response> response)
{
    std::lock_guard<std::mutex> lock(queue_mutex_);

    // 排队中的订单
    for (const auto & record : order_queue_)
    {
        DeliveryStatus status;
        status.order_id = record.order.order_id;
        status.state = state_to_msg(record.state);
        status.error_msg = record.error_msg;
        response->reports.push_back(status);
    }

    // 已完成/失败的订单
    for (const auto & record : completed_orders_)
    {
        DeliveryStatus status;
        status.order_id = record.order.order_id;
        status.state = state_to_msg(record.state);
        status.error_msg = record.error_msg;
        response->reports.push_back(status);
    }
}

// --- 配置加载 ---

bool DeliveryManager::load_station_config(const std::string & path)
{
    if (path.empty())
    {
        RCLCPP_ERROR(get_logger(), "站点配置路径为空");
        return false;
    }

    YAML::Node root;
    try
    {
        root = YAML::LoadFile(path);
    }
    catch (const std::exception & e)
    {
        RCLCPP_ERROR(get_logger(), "YAML 解析错误: %s", e.what());
        return false;
    }

    const auto stations_node = root["stations"];
    if (!stations_node || !stations_node.IsSequence())
    {
        RCLCPP_ERROR(get_logger(), "YAML 中缺少 'stations' 列表");
        return false;
    }

    stations_.clear();
    for (size_t i = 0; i < stations_node.size(); ++i)
    {
        const auto & node = stations_node[i];
        Station station;
        station.id = node["station_id"].as<std::string>();
        station.pose.x = node["x"].as<double>();
        station.pose.y = node["y"].as<double>();
        station.pose.yaw = node["yaw"] ? node["yaw"].as<double>() : 0.0;
        station.type = node["station_type"] ? node["station_type"].as<uint8_t>() : 0;

        stations_[station.id] = station;
        RCLCPP_INFO(get_logger(), "  站点 [%s]: (%.2f, %.2f, %.2f) 类型=%u",
                    station.id.c_str(), station.pose.x, station.pose.y,
                    station.pose.yaw, station.type);
    }

    return !stations_.empty();
}

// --- 环境准备（复用 task_runner 模式） ---

bool DeliveryManager::wait_for_time()
{
    const auto start_wall = std::chrono::steady_clock::now();
    while (rclcpp::ok())
    {
        if (this->get_clock()->now().nanoseconds() != 0)
        {
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

bool DeliveryManager::wait_for_tf()
{
    tf2_ros::Buffer tf_buffer(this->get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer);

    RCLCPP_INFO(get_logger(), "等待 TF 链路: [%s] → [%s]...",
                map_frame_.c_str(), base_frame_.c_str());

    const auto start_time = this->now();
    while (rclcpp::ok())
    {
        try
        {
            tf_buffer.lookupTransform(map_frame_, base_frame_,
                                      tf2::TimePointZero);
            RCLCPP_INFO(get_logger(), "TF 链路就绪");
            return true;
        }
        catch (const tf2::TransformException &)
        {
            rclcpp::sleep_for(200ms);
        }

        if ((this->now() - start_time).seconds() > tf_wait_timeout_sec_)
        {
            return false;
        }
    }
    return false;
}

bool DeliveryManager::wait_for_action_server()
{
    RCLCPP_INFO(get_logger(), "等待 Nav2 action server [%s]...",
                nav2_action_name_.c_str());
    return action_client_->wait_for_action_server(10s);
}

void DeliveryManager::publish_initial_pose()
{
    PoseWithCovarianceStamped msg;
    msg.header.frame_id = map_frame_;
    msg.header.stamp = this->now();
    msg.pose.pose.position.x = initial_x_;
    msg.pose.pose.position.y = initial_y_;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, initial_yaw_);
    msg.pose.pose.orientation = tf2::toMsg(q);

    msg.pose.covariance[0] = 0.25;
    msg.pose.covariance[7] = 0.25;
    msg.pose.covariance[35] = 0.25;

    RCLCPP_INFO(get_logger(), "发布初始位姿到 /initialpose");
    for (int i = 0; rclcpp::ok() && i < 5; ++i)
    {
        msg.header.stamp = this->now();
        initial_pose_pub_->publish(msg);
        rclcpp::sleep_for(200ms);
    }
}

// --- 导航（复用 task_runner 模式）---

DeliveryManager::PoseStamped DeliveryManager::make_pose(const Pose2D & pose) const
{
    PoseStamped msg;
    msg.header.frame_id = map_frame_;
    msg.header.stamp = this->now();
    msg.pose.position.x = pose.x;
    msg.pose.position.y = pose.y;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, pose.yaw);
    msg.pose.orientation = tf2::toMsg(q);

    return msg;
}

bool DeliveryManager::navigate_to(const Pose2D & pose, const std::string & label)
{
    NavigateToPose::Goal goal_msg;
    goal_msg.pose = make_pose(pose);

    auto send_goal_options =
        rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    send_goal_options.feedback_callback =
        [this, label](GoalHandle::SharedPtr,
                      const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                             "[%s] 剩余距离: %.2f 米", label.c_str(),
                             feedback->distance_remaining);
    };

    auto goal_future =
        action_client_->async_send_goal(goal_msg, send_goal_options);

    // 等待 goal 被接受（executor 在后台 spin，无需手动 spin）
    if (goal_future.wait_for(5s) != std::future_status::ready)
    {
        RCLCPP_ERROR(get_logger(), "[%s] 目标发送超时", label.c_str());
        return false;
    }

    auto goal_handle = goal_future.get();
    if (!goal_handle)
    {
        RCLCPP_ERROR(get_logger(), "[%s] 导航目标被拒绝", label.c_str());
        return false;
    }

    auto result_future = action_client_->async_get_result(goal_handle);
    const std::chrono::duration<double> timeout(navigation_timeout_sec_);

    if (result_future.wait_for(timeout) == std::future_status::ready)
    {
        const auto wrapped_result = result_future.get();
        return (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED);
    }

    RCLCPP_WARN(get_logger(), "[%s] 导航超时，发送取消指令...", label.c_str());
    auto cancel_future = action_client_->async_cancel_goal(goal_handle);
    cancel_future.wait_for(5s);
    return false;
}

// --- 状态管理 ---

void DeliveryManager::publish_status(
    const std::string & order_id, DeliveryState state,
    const std::string & station, float progress, const std::string & error)
{
    DeliveryStatus msg;
    msg.order_id = order_id;
    msg.state = state_to_msg(state);
    msg.current_station = station;
    msg.progress = progress;
    msg.error_msg = error;
    status_pub_->publish(msg);

    RCLCPP_INFO(get_logger(), "状态更新 [%s]: %s (站点: %s, 进度: %.0f%%)",
                order_id.c_str(), state_to_string(state).c_str(),
                station.c_str(), progress * 100.0f);
}

uint8_t DeliveryManager::state_to_msg(DeliveryState state) const
{
    switch (state)
    {
    case DeliveryState::kIdle:           return DeliveryStatus::STATE_IDLE;
    case DeliveryState::kGoingToPickup:  return DeliveryStatus::STATE_GOING_TO_PICKUP;
    case DeliveryState::kWaitingLoad:    return DeliveryStatus::STATE_WAITING_LOAD;
    case DeliveryState::kGoingToDropoff: return DeliveryStatus::STATE_GOING_TO_DROPOFF;
    case DeliveryState::kWaitingUnload:  return DeliveryStatus::STATE_WAITING_UNLOAD;
    case DeliveryState::kComplete:       return DeliveryStatus::STATE_COMPLETE;
    case DeliveryState::kFailed:         return DeliveryStatus::STATE_FAILED;
    default:                             return DeliveryStatus::STATE_IDLE;
    }
}

std::string DeliveryManager::state_to_string(DeliveryState state) const
{
    switch (state)
    {
    case DeliveryState::kIdle:           return "空闲(Idle)";
    case DeliveryState::kGoingToPickup:  return "前往取货点(GoingToPickup)";
    case DeliveryState::kWaitingLoad:    return "等待装货(WaitingLoad)";
    case DeliveryState::kGoingToDropoff: return "前往送货点(GoingToDropoff)";
    case DeliveryState::kWaitingUnload:  return "等待卸货(WaitingUnload)";
    case DeliveryState::kComplete:       return "配送完成(Complete)";
    case DeliveryState::kFailed:         return "配送失败(Failed)";
    default:                             return "未知(Unknown)";
    }
}

}  // namespace delivery_core

/**
 * @file delivery_executor.cpp
 * @brief 配送执行器生命周期节点实现。
 *
 * Phase 3: 转为 LifecycleNode。
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

#include "yaml-cpp/yaml.h"

#include <chrono>
#include <functional>
#include <thread>

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

// ======================== 生命周期回调 ========================

DeliveryExecutor::CallbackReturn DeliveryExecutor::on_configure(
    const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "on_configure: 开始配置...");

    // 创建回调组
    service_cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::Reentrant);

    // 创建辅助 Node 供 BT 节点使用（LifecycleNode 不继承 rclcpp::Node）
    bt_node_ = rclcpp::Node::make_shared(
        "delivery_executor_bt_helper",
        this->get_namespace());

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
    if (!load_station_config(station_config_path_))
    {
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
    RCLCPP_INFO(get_logger(), "on_activate: 创建 Action Server...");

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

    // 销毁 Action Server
    action_server_.reset();

    RCLCPP_INFO(get_logger(), "on_deactivate: 完成");
    return CallbackReturn::SUCCESS;
}

DeliveryExecutor::CallbackReturn DeliveryExecutor::on_cleanup(
    const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "on_cleanup: 清理资源...");

    // 停止辅助节点的 spin 线程
    if (bt_node_executor_)
    {
        bt_node_executor_->cancel();
    }
    if (bt_node_spin_thread_.joinable())
    {
        bt_node_spin_thread_.join();
    }
    bt_node_executor_.reset();

    stations_.clear();
    bt_node_.reset();
    nav_client_.reset();
    status_pub_.reset();
    cmd_vel_pub_.reset();
    confirm_load_srv_.reset();
    confirm_unload_srv_.reset();
    battery_level_ = 100.0;

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
    action_server_.reset();

    // 停止辅助节点的 spin 线程
    if (bt_node_executor_)
    {
        bt_node_executor_->cancel();
    }
    if (bt_node_spin_thread_.joinable())
    {
        bt_node_spin_thread_.join();
    }
    bt_node_executor_.reset();

    return CallbackReturn::SUCCESS;
}

// ======================== 配置加载 ========================

bool DeliveryExecutor::load_station_config(const std::string & path)
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

        if (station.type > 2)
        {
            RCLCPP_ERROR(get_logger(), "站点 [%s] 类型非法: %u",
                         station.id.c_str(), station.type);
            return false;
        }

        if (stations_.count(station.id) > 0)
        {
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

    // 单任务模式：同时只接受一个配送任务
    if (executing_.load())
    {
        RCLCPP_WARN(get_logger(), "拒绝配送请求 [%s]：当前有任务在执行中",
                    order.order_id.c_str());
        return rclcpp_action::GoalResponse::REJECT;
    }

    // 验证站点存在
    if (stations_.find(order.pickup_station) == stations_.end() ||
        stations_.find(order.dropoff_station) == stations_.end())
    {
        RCLCPP_WARN(get_logger(), "拒绝配送请求 [%s]：站点不存在",
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
    // 在新线程中执行 BT，避免阻塞 executor 线程
    std::thread([this, goal_handle]() {
        execute_bt(goal_handle);
    }).detach();
}

// ======================== BT 执行核心循环 ========================

void DeliveryExecutor::execute_bt(
    const std::shared_ptr<GoalHandleExecuteDelivery> goal_handle)
{
    executing_.store(true);

    const auto & order = goal_handle->get_goal()->order;
    const auto start_time = std::chrono::steady_clock::now();

    RCLCPP_INFO(get_logger(), "开始执行配送 [%s]", order.order_id.c_str());

    // 重置确认标志，确保每次新订单从干净状态开始
    load_confirmed_.store(false, std::memory_order_release);
    unload_confirmed_.store(false, std::memory_order_release);

    // 从 XML 文件创建行为树实例
    BT::Tree tree;
    try
    {
        tree = factory_.createTreeFromFile(tree_file_path_);
    }
    catch (const std::exception & e)
    {
        RCLCPP_ERROR(get_logger(), "创建行为树失败: %s", e.what());

        auto result = std::make_shared<ExecuteDelivery::Result>();
        result->success = false;
        result->error_msg = std::string("BT 创建失败: ") + e.what();
        goal_handle->abort(result);
        executing_.store(false);
        return;
    }

    // 设置黑板变量
    tree.rootBlackboard()->set("order_id", order.order_id);
    tree.rootBlackboard()->set("pickup_station", order.pickup_station);
    tree.rootBlackboard()->set("dropoff_station", order.dropoff_station);
    tree.rootBlackboard()->set("stations", stations_);
    tree.rootBlackboard()->set("battery_level", battery_level_);

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

    while (rclcpp::ok() && bt_status == BT::NodeStatus::RUNNING)
    {
        // 检查取消请求
        if (goal_handle->is_canceling())
        {
            RCLCPP_INFO(get_logger(), "配送被取消 [%s]", order.order_id.c_str());
            tree.haltTree();

            auto result = std::make_shared<ExecuteDelivery::Result>();
            result->success = false;
            result->error_msg = "配送被取消";

            const auto elapsed = std::chrono::steady_clock::now() - start_time;
            result->elapsed_time_sec =
                std::chrono::duration_cast<std::chrono::duration<double>>(elapsed).count();

            goal_handle->canceled(result);
            executing_.store(false);
            return;
        }

        // tick 一次行为树
        bt_status = tree.tickOnce();

        // 发布 feedback
        auto feedback = std::make_shared<ExecuteDelivery::Feedback>();
        feedback->state = DeliveryStatus::STATE_GOING_TO_PICKUP;
        feedback->progress = 0.5f;
        goal_handle->publish_feedback(feedback);

        rate.sleep();
    }

    // 构造结果
    auto result = std::make_shared<ExecuteDelivery::Result>();
    const auto elapsed = std::chrono::steady_clock::now() - start_time;
    result->elapsed_time_sec =
        std::chrono::duration_cast<std::chrono::duration<double>>(elapsed).count();

    if (bt_status == BT::NodeStatus::SUCCESS)
    {
        result->success = true;

        // 配送完成后扣减电量
        battery_level_ -= battery_drain_per_delivery_;
        if (battery_level_ < 0.0) battery_level_ = 0.0;
        RCLCPP_INFO(get_logger(), "配送完成 [%s]，耗时 %.1f 秒，剩余电量 %.1f%%",
                    order.order_id.c_str(), result->elapsed_time_sec,
                    battery_level_);
        goal_handle->succeed(result);
    }
    else
    {
        result->success = false;
        result->error_msg = "行为树执行失败";
        RCLCPP_WARN(get_logger(), "配送失败 [%s]", order.order_id.c_str());

        // 发布失败状态
        DeliveryStatus fail_msg;
        fail_msg.stamp = this->now();
        fail_msg.order_id = order.order_id;
        fail_msg.state = DeliveryStatus::STATE_FAILED;
        fail_msg.error_msg = result->error_msg;
        status_pub_->publish(fail_msg);

        goal_handle->abort(result);
    }

    executing_.store(false);
}

}  // namespace delivery_core

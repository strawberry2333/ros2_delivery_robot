#pragma once

/**
 * @file delivery_executor.hpp
 * @brief 配送执行器节点声明（BT 宿主）。
 *
 * 作为 ExecuteDelivery Action Server，接收 delivery_manager 发来的配送请求，
 * 使用 BehaviorTree.CPP 驱动行为树执行完整的配送流程。
 *
 * 架构：
 *   delivery_manager (Action Client) → /execute_delivery → delivery_executor (Action Server)
 *     └→ BehaviorTree: NavigateToStation → DockAtStation → ReportStatus → WaitForConfirmation × 2
 */

#include <atomic>
#include <mutex>
#include <string>
#include <unordered_map>

#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "delivery_interfaces/action/execute_delivery.hpp"
#include "delivery_interfaces/msg/delivery_order.hpp"
#include "delivery_interfaces/msg/delivery_status.hpp"

#include "delivery_core/nodes/navigate_to_station.hpp"

namespace delivery_core
{

/**
 * @brief 配送执行器节点
 *
 * 核心职责：
 * 1. 作为 /execute_delivery Action Server 接收配送请求
 * 2. 从 YAML 加载站点配置
 * 3. 注册 BT 节点并从 XML 文件创建行为树
 * 4. 驱动 BT tick 循环完成配送
 * 5. 持有 confirm_load/confirm_unload 服务端
 */
class DeliveryExecutor : public rclcpp::Node
{
public:
    using ExecuteDelivery = delivery_interfaces::action::ExecuteDelivery;
    using GoalHandleExecuteDelivery = rclcpp_action::ServerGoalHandle<ExecuteDelivery>;
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using DeliveryStatus = delivery_interfaces::msg::DeliveryStatus;

    DeliveryExecutor();

private:
    // ====== 配置加载 ======
    bool load_station_config(const std::string & path);

    // ====== BT 工厂初始化 ======
    void register_bt_nodes();

    // ====== Action Server 回调 ======
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const ExecuteDelivery::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleExecuteDelivery> goal_handle);

    void handle_accepted(
        const std::shared_ptr<GoalHandleExecuteDelivery> goal_handle);

    // ====== BT 执行 ======
    void execute_bt(const std::shared_ptr<GoalHandleExecuteDelivery> goal_handle);

    // ====== ROS 参数 ======
    std::string station_config_path_;
    std::string tree_file_path_;

    // ====== 站点数据 ======
    StationMap stations_;

    // ====== BT 工厂 ======
    BT::BehaviorTreeFactory factory_;

    // ====== 确认标志 ======
    std::atomic<bool> load_confirmed_{false};
    std::atomic<bool> unload_confirmed_{false};

    // ====== 执行状态 ======
    std::atomic<bool> executing_{false};

    // ====== ROS 通信接口 ======
    rclcpp_action::Server<ExecuteDelivery>::SharedPtr action_server_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    rclcpp::Publisher<DeliveryStatus>::SharedPtr status_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    // ====== 确认服务端 ======
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr confirm_load_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr confirm_unload_srv_;

    // ====== 回调组 ======
    rclcpp::CallbackGroup::SharedPtr service_cb_group_;
};

}  // namespace delivery_core

#pragma once

/**
 * @file navigate_to_station.hpp
 * @brief BT 叶节点：导航到指定站点。
 *
 * 从黑板获取 station_id，查找站点坐标，调用 Nav2 NavigateToPose Action 完成导航。
 * 参考 Ros2Learning/move_base_node.cpp 的 StatefulActionNode 模式。
 */

#include <string>
#include <unordered_map>

#include "behaviortree_cpp/action_node.h"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace delivery_core
{

/// 2D 位姿，用于站点坐标存储
struct Pose2D
{
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
};

/// 站点描述
struct Station
{
    std::string id;
    Pose2D pose;
    uint8_t type{0};
};

/// 站点映射表类型别名
using StationMap = std::unordered_map<std::string, Station>;

/**
 * @brief 导航到指定站点的 BT 叶节点（StatefulActionNode）
 *
 * 输入端口：station_id — 目标站点 ID
 * 从黑板 "stations" 获取站点坐标映射表
 */
class NavigateToStation : public BT::StatefulActionNode
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    NavigateToStation(
        const std::string & name,
        const BT::NodeConfig & config,
        rclcpp::Node::SharedPtr node,
        rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client);

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<std::string>("station_id", "目标站点 ID")};
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;

    GoalHandle::SharedPtr goal_handle_;
    std::shared_future<GoalHandle::WrappedResult> result_future_;
    bool result_ready_{false};
};

}  // namespace delivery_core

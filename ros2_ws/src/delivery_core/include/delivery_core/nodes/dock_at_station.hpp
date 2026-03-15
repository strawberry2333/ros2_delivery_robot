#pragma once

/**
 * @file dock_at_station.hpp
 * @brief BT 叶节点：模拟停靠对接。
 *
 * 到达站点后发布低速 cmd_vel 模拟微调停靠动作。
 */

#include <string>

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace delivery_core
{

/**
 * @brief 模拟停靠对接的 BT 叶节点（SyncActionNode）
 *
 * 输入端口：station_id — 停靠站点 ID（用于日志）
 * 发布 cmd_vel 前进 0.5 秒模拟停靠
 */
class DockAtStation : public BT::SyncActionNode
{
public:
    DockAtStation(
        const std::string & name,
        const BT::NodeConfig & config,
        rclcpp::Node::SharedPtr node,
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub);

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<std::string>("station_id", "停靠站点 ID")};
    }

    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};

}  // namespace delivery_core

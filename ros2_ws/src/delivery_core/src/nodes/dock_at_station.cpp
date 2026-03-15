/**
 * @file dock_at_station.cpp
 * @brief DockAtStation BT 叶节点实现。
 *
 * 模拟停靠动作：发布低速 cmd_vel 前进 0.5 秒。
 */

#include "delivery_core/nodes/dock_at_station.hpp"

#include <chrono>
#include <thread>

namespace delivery_core
{

DockAtStation::DockAtStation(
    const std::string & name,
    const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node,
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub)
    : BT::SyncActionNode(name, config),
      node_(node),
      cmd_vel_pub_(cmd_vel_pub)
{
}

BT::NodeStatus DockAtStation::tick()
{
    std::string station_id;
    getInput("station_id", station_id);

    RCLCPP_INFO(node_->get_logger(), "DockAtStation: 停靠对接 [%s]", station_id.c_str());

    // 发布低速前进指令，模拟微调停靠
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.05;

    // 以 10Hz 发布 0.5 秒
    for (int i = 0; i < 5; ++i)
    {
        cmd_vel_pub_->publish(cmd);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // 发布停止指令
    cmd.linear.x = 0.0;
    cmd_vel_pub_->publish(cmd);

    RCLCPP_INFO(node_->get_logger(), "DockAtStation: 停靠完成 [%s]", station_id.c_str());
    return BT::NodeStatus::SUCCESS;
}

}  // namespace delivery_core

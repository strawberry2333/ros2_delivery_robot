/**
 * @file dock_at_station.cpp
 * @brief DockAtStation BT 叶节点实现。
 *
 * 使用 StatefulActionNode 模式，每次 tick 发布一帧低速 cmd_vel，
 * 5 帧后完成停靠（约 0.5s @10Hz tick）。可通过 onHalted 随时中断。
 */

#include "delivery_core/nodes/dock_at_station.hpp"

namespace delivery_core
{

DockAtStation::DockAtStation(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node,
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub)
: BT::StatefulActionNode(name, config),
  node_(node),
  cmd_vel_pub_(cmd_vel_pub)
{
}

BT::NodeStatus DockAtStation::onStart()
{
  std::string station_id;
  getInput("station_id", station_id);
  RCLCPP_INFO(node_->get_logger(), "DockAtStation: 停靠对接 [%s]", station_id.c_str());

  iteration_count_ = 0;

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.05;
  cmd_vel_pub_->publish(cmd);
  ++iteration_count_;

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DockAtStation::onRunning()
{
  if (iteration_count_ >= kMaxIterations) {
    stop_robot();

    std::string station_id;
    getInput("station_id", station_id);
    RCLCPP_INFO(node_->get_logger(), "DockAtStation: 停靠完成 [%s]", station_id.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.05;
  cmd_vel_pub_->publish(cmd);
  ++iteration_count_;

  return BT::NodeStatus::RUNNING;
}

void DockAtStation::onHalted()
{
  stop_robot();
  RCLCPP_INFO(node_->get_logger(), "DockAtStation: 停靠被中断");
}

void DockAtStation::stop_robot()
{
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.0;
  cmd_vel_pub_->publish(cmd);
}

}  // namespace delivery_core

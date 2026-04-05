#pragma once

/**
 * @file dock_at_station.hpp
 * @brief BT 叶节点：模拟停靠对接。
 *
 * 这个节点处在"机器人已经到站，但还需要微调对位"的阶段。
 * 使用 StatefulActionNode 实现，每次 tick 发一次 cmd_vel，
 * 支持通过 halt 中断停靠过程。
 *
 * 业务语义：
 * - `RUNNING`：停靠尚未完成，低速前进中
 * - `SUCCESS`：停靠动作完成，可以进入装货/卸货确认阶段
 */

#include <string>

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace delivery_core
{

/**
 * @brief 模拟停靠对接的 BT 叶节点（StatefulActionNode）
 *
 * 输入端口：
 * - `station_id`：当前停靠的站点 ID，仅用于日志和调试输出
 *
 * 每次 tick 发布一帧低速 cmd_vel，50 帧后完成停靠（约 500ms @100Hz）。
 * 可通过 onHalted 随时中断并刹停。
 */
class DockAtStation : public BT::StatefulActionNode
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

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void stop_robot();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  int iteration_count_{0};
  static constexpr int kMaxIterations = 50;
};

}  // namespace delivery_core

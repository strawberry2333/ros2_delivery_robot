/**
 * @file dock_at_station.cpp
 * @brief DockAtStation BT 叶节点实现。
 *
 * 这是靠站后的“最后一小段动作”节点，目的是让 demo 看起来像真的在
 * 进行停靠，而不是导航到站点后直接切换到确认阶段。
 *
 * 业务语义：
 * - `SUCCESS`：停靠动作完成，可以开始装货/卸货确认
 * - `FAILURE`：当前实现不主动失败；未来如果加入底盘异常检测，可在这里失败退出
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
  // 构造阶段只保存依赖对象，真正的停靠动作发生在 tick()。
}

BT::NodeStatus DockAtStation::tick()
{
  // station_id 只用于日志，不参与控制逻辑。
  std::string station_id;
  getInput("station_id", station_id);

  // 先打日志，方便从执行轨迹里看出当前靠的是哪个站点。
  RCLCPP_INFO(node_->get_logger(), "DockAtStation: 停靠对接 [%s]", station_id.c_str());

  // 发布低速前进指令，模拟微调停靠。
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.05;

  // 以固定频率持续短时间发布，模拟机器人缓慢贴近站点边缘。
  for (int i = 0; i < 5; ++i) {
    cmd_vel_pub_->publish(cmd);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // 动作结束后必须显式刹停，避免速度命令残留。
  cmd.linear.x = 0.0;
  cmd_vel_pub_->publish(cmd);

  // 这里直接成功，表示“靠站动作”已经完成。
  RCLCPP_INFO(node_->get_logger(), "DockAtStation: 停靠完成 [%s]", station_id.c_str());
  return BT::NodeStatus::SUCCESS;
}

}  // namespace delivery_core

#pragma once

/**
 * @file dock_at_station.hpp
 * @brief BT 叶节点：模拟停靠对接。
 *
 * 这个节点处在“机器人已经到站，但还需要微调对位”的阶段。
 * 在真实项目里，这里往往会对应末端精定位、自动对接桩或货架前沿贴靠；
 * 当前仓库使用的是 demo 级实现，用低速 `cmd_vel` 模拟短暂前进。
 *
 * 业务语义：
 * - `SUCCESS`：停靠动作完成，可以进入装货/卸货确认阶段
 * - `FAILURE`：当前实现中不会主动失败，若未来加入底盘控制异常再考虑失败分支
 *
 * @note 当前为 demo 实现：阻塞式 cmd_vel 模拟停靠
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
 * 输入端口：
 * - `station_id`：当前停靠的站点 ID，仅用于日志和调试输出
 *
 * 这个节点是同步动作节点，因为“模拟停靠”只需要发布一小段固定时长的
 * 速度命令，不依赖异步 action 结果。行为树 tick 到这里时会阻塞到动作结束。
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
    // 站点 ID 不参与控制逻辑，但会影响日志可读性，便于定位当前任务阶段。
    return {BT::InputPort<std::string>("station_id", "停靠站点 ID")};
  }

  /**
   * @brief 执行停靠动作。
   *
   * 这里的“停靠”本质上是一个短时速度脉冲：先低速前进，再显式刹停。
   * 这样做的目的是让 demo 具有接近真实机器人靠站的视觉效果，同时不引入
   * 复杂的底盘控制逻辑。
   */
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};

}  // namespace delivery_core

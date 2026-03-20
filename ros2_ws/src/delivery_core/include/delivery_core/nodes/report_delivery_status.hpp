#pragma once

/**
 * @file report_delivery_status.hpp
 * @brief BT 叶节点：发布配送状态消息。
 *
 * 这个节点通常出现在行为树的阶段切换点，用来把当前订单进度显式同步到
 * `DeliveryStatus` 话题和黑板。外部监控、`delivery_manager` 的反馈同步、
 * 以及日志排查都依赖这个状态输出。
 *
 * 业务语义：
 * - `SUCCESS`：状态已成功发布，行为树可以继续
 * - `FAILURE`：必要端口缺失，当前状态无法对外同步
 */

#include <string>

#include "behaviortree_cpp/action_node.h"
#include "delivery_interfaces/msg/delivery_status.hpp"
#include "rclcpp/rclcpp.hpp"

namespace delivery_core
{

/**
 * @brief 发布配送状态的 BT 叶节点（SyncActionNode）
 *
 * 输入端口：
 * - order_id: 订单 ID
 * - state: 状态值 (uint8)
 * - station_id: 当前站点 ID
 * - progress: 进度值 (0.0 ~ 1.0)
 *
 * 输出端口：
 * - bt_state: 写回黑板的状态值，供父树或 executor 读取
 * - bt_station: 写回黑板的当前站点
 * - bt_progress: 写回黑板的当前进度
 *
 * 这个节点是同步动作节点，因为它只做消息构造和发布，不需要等待外部结果。
 */
class ReportDeliveryStatus : public BT::SyncActionNode
{
public:
  using DeliveryStatus = delivery_interfaces::msg::DeliveryStatus;

  ReportDeliveryStatus(
    const std::string & name,
    const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node,
    rclcpp::Publisher<DeliveryStatus>::SharedPtr status_pub);

  static BT::PortsList providedPorts()
  {
    return {
      // 订单 ID 和状态值是最小必需字段，没有它们就无法形成有效状态消息。
      BT::InputPort<std::string>("order_id", "订单 ID"),
      BT::InputPort<unsigned>("state", "状态值"),
      BT::InputPort<std::string>("station_id", "站点 ID"),
      BT::InputPort<double>("progress", "进度 0.0~1.0"),
      // 输出端口用于把当前状态回写黑板，让父树或宿主节点读取。
      BT::OutputPort<unsigned>("bt_state", "当前状态值（写入黑板）"),
      BT::OutputPort<std::string>("bt_station", "当前站点（写入黑板）"),
      BT::OutputPort<float>("bt_progress", "当前进度（写入黑板）"),
    };
  }

  /**
   * @brief 构造并发布状态消息，同时把关键字段写回黑板。
   *
   * 这是“状态同步节点”，不是纯日志节点。它既要把状态发到话题上，也要
   * 保证黑板中有可供上层读取的最新状态，便于 action feedback 和报告汇总。
   */
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<DeliveryStatus>::SharedPtr status_pub_;
};

}  // namespace delivery_core

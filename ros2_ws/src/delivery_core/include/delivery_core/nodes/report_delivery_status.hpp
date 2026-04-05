#pragma once

/**
 * @file report_delivery_status.hpp
 * @brief BT 叶节点：同步配送状态到黑板。
 *
 * 这个节点出现在行为树的阶段切换点，把当前订单进度写入黑板。
 * executor 的 tick 循环读取黑板后通过 action feedback 转发给 manager，
 * manager 作为唯一对外发布者将状态发布到 /delivery_status 话题。
 *
 * 业务语义：
 * - `SUCCESS`：状态已写入黑板，行为树可以继续
 * - `FAILURE`：必要端口缺失，当前状态无法同步
 */

#include <string>

#include "behaviortree_cpp/action_node.h"
#include "delivery_interfaces/msg/delivery_status.hpp"
#include "rclcpp/rclcpp.hpp"

namespace delivery_core
{

/**
 * @brief 同步配送状态到黑板的 BT 叶节点（SyncActionNode）
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
 * 同步动作节点，只做黑板写入，不需要等待外部结果。
 */
class ReportDeliveryStatus : public BT::SyncActionNode
{
public:
  using DeliveryStatus = delivery_interfaces::msg::DeliveryStatus;

  ReportDeliveryStatus(
    const std::string & name,
    const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node);

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
   * @brief 将关键字段写回黑板，供 executor 通过 action feedback 转发。
   */
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace delivery_core

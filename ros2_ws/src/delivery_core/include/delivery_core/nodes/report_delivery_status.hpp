#pragma once

/**
 * @file report_delivery_status.hpp
 * @brief BT 叶节点：发布配送状态消息。
 *
 * 在行为树执行的关键节点发布 DeliveryStatus，
 * 使外部监控系统能追踪配送进度。
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
      BT::InputPort<std::string>("order_id", "订单 ID"),
      BT::InputPort<unsigned>("state", "状态值"),
      BT::InputPort<std::string>("station_id", "站点 ID"),
      BT::InputPort<double>("progress", "进度 0.0~1.0"),
            // 输出端口：将状态写入黑板，供 executor 读取作为 feedback
      BT::OutputPort<unsigned>("bt_state", "当前状态值（写入黑板）"),
      BT::OutputPort<std::string>("bt_station", "当前站点（写入黑板）"),
      BT::OutputPort<float>("bt_progress", "当前进度（写入黑板）"),
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<DeliveryStatus>::SharedPtr status_pub_;
};

}  // namespace delivery_core

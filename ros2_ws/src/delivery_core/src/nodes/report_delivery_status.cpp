/**
 * @file report_delivery_status.cpp
 * @brief ReportDeliveryStatus BT 叶节点实现。
 *
 * 这个节点把行为树内部的阶段状态同步到外部世界：
 * 一方面发布 `DeliveryStatus`，一方面把状态写回黑板，供宿主节点读取。
 */

#include "delivery_core/nodes/report_delivery_status.hpp"

namespace delivery_core
{

ReportDeliveryStatus::ReportDeliveryStatus(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node,
  rclcpp::Publisher<DeliveryStatus>::SharedPtr status_pub)
: BT::SyncActionNode(name, config),
  node_(node),
  status_pub_(status_pub)
{
  // 构造阶段只保存依赖，不做任何发布动作。
}

BT::NodeStatus ReportDeliveryStatus::tick()
{
  // order_id 和 state 是状态消息的最小必需字段。
  std::string order_id;
  unsigned state = 0;
  std::string station_id;
  double progress = 0.0;

  if (!getInput("order_id", order_id) || !getInput("state", state)) {
    RCLCPP_ERROR(node_->get_logger(), "ReportDeliveryStatus: 缺少必要端口");
    return BT::NodeStatus::FAILURE;
  }

  // 站点和进度是增强信息，便于可视化和外部监控；缺失时仍可继续发布。
  getInput("station_id", station_id);
  getInput("progress", progress);

  // 构造对外发布的状态消息。
  DeliveryStatus msg;
  msg.stamp = node_->now();
  msg.order_id = order_id;
  msg.state = static_cast<uint8_t>(state);
  msg.current_station = station_id;
  msg.progress = static_cast<float>(progress);

  // 先发布到话题，让订阅方实时看到状态变化。
  status_pub_->publish(msg);

  // 再回写黑板，确保父树和 executor 能读取到最新阶段信息。
  setOutput("bt_state", state);
  setOutput("bt_station", station_id);
  setOutput("bt_progress", static_cast<float>(progress));

  // 日志以百分比形式展示进度，更适合人工排查。
  RCLCPP_INFO(node_->get_logger(),
        "ReportDeliveryStatus: [%s] state=%u station=%s progress=%.0f%%",
        order_id.c_str(), state, station_id.c_str(), progress * 100.0);

  // 这是纯同步状态同步节点，发布成功即可返回 SUCCESS。
  return BT::NodeStatus::SUCCESS;
}

}  // namespace delivery_core

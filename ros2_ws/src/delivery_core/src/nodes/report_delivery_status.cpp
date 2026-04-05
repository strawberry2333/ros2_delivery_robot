/**
 * @file report_delivery_status.cpp
 * @brief ReportDeliveryStatus BT 叶节点实现。
 *
 * 这个节点把行为树内部的阶段状态写回黑板，供宿主节点（executor）
 * 通过 action feedback 转发给 delivery_manager。
 * 对外 /delivery_status 话题的发布由 manager 统一负责。
 */

#include "delivery_core/nodes/report_delivery_status.hpp"

namespace delivery_core
{

ReportDeliveryStatus::ReportDeliveryStatus(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::SyncActionNode(name, config),
  node_(node)
{
}

BT::NodeStatus ReportDeliveryStatus::tick()
{
  std::string order_id;
  unsigned state = 0;
  std::string station_id;
  double progress = 0.0;

  if (!getInput("order_id", order_id) || !getInput("state", state)) {
    RCLCPP_ERROR(node_->get_logger(), "ReportDeliveryStatus: 缺少必要端口");
    return BT::NodeStatus::FAILURE;
  }

  getInput("station_id", station_id);
  getInput("progress", progress);

  // 回写黑板，executor tick 循环读取后通过 action feedback 转发给 manager。
  setOutput("bt_state", state);
  setOutput("bt_station", station_id);
  setOutput("bt_progress", static_cast<float>(progress));

  RCLCPP_INFO(node_->get_logger(),
        "ReportDeliveryStatus: [%s] state=%u station=%s progress=%.0f%%",
        order_id.c_str(), state, station_id.c_str(), progress * 100.0);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace delivery_core

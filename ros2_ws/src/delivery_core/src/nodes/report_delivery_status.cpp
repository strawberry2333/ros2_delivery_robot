/**
 * @file report_delivery_status.cpp
 * @brief ReportDeliveryStatus BT 叶节点实现。
 *
 * 构造并发布 DeliveryStatus 消息到 /delivery_status 话题。
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
}

BT::NodeStatus ReportDeliveryStatus::tick()
{
    std::string order_id;
    unsigned state = 0;
    std::string station_id;
    double progress = 0.0;

    if (!getInput("order_id", order_id) || !getInput("state", state))
    {
        RCLCPP_ERROR(node_->get_logger(), "ReportDeliveryStatus: 缺少必要端口");
        return BT::NodeStatus::FAILURE;
    }

    getInput("station_id", station_id);
    getInput("progress", progress);

    DeliveryStatus msg;
    msg.stamp = node_->now();
    msg.order_id = order_id;
    msg.state = static_cast<uint8_t>(state);
    msg.current_station = station_id;
    msg.progress = static_cast<float>(progress);

    status_pub_->publish(msg);

    RCLCPP_INFO(node_->get_logger(),
        "ReportDeliveryStatus: [%s] state=%u station=%s progress=%.0f%%",
        order_id.c_str(), state, station_id.c_str(), progress * 100.0);

    return BT::NodeStatus::SUCCESS;
}

}  // namespace delivery_core

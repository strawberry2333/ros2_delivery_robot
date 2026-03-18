/**
 * @file test_report_delivery_status.cpp
 * @brief ReportDeliveryStatus BT 节点单元测试。
 *
 * 测试场景：
 * 1. 正确发布 DeliveryStatus 消息
 * 2. 节点返回 SUCCESS
 */

#include <gtest/gtest.h>

#include "behaviortree_cpp/bt_factory.h"
#include "delivery_core/nodes/report_delivery_status.hpp"
#include "delivery_interfaces/msg/delivery_status.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace delivery_core;
using DeliveryStatus = delivery_interfaces::msg::DeliveryStatus;

class ReportDeliveryStatusTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = rclcpp::Node::make_shared("test_report_node");
    status_pub_ = node_->create_publisher<DeliveryStatus>("delivery_status", 10);

    factory_.registerBuilder<ReportDeliveryStatus>(
            "ReportDeliveryStatus",
      [this](const std::string & name, const BT::NodeConfig & config) {
        return std::make_unique<ReportDeliveryStatus>(
                    name, config, node_, status_pub_);
            });
  }

  void TearDown() override
  {
    node_.reset();
    rclcpp::shutdown();
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<DeliveryStatus>::SharedPtr status_pub_;
  BT::BehaviorTreeFactory factory_;
};

TEST_F(ReportDeliveryStatusTest, PublishesStatusAndReturnsSuccess)
{
    const std::string xml =
    R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="Test">
        <ReportDeliveryStatus order_id="test_order" state="5"
                              station_id="station_C" progress="1.0" />
      </BehaviorTree>
    </root>
    )";

    // 订阅以捕获发布的消息
    DeliveryStatus received_msg;
    bool received = false;
    auto sub = node_->create_subscription<DeliveryStatus>(
        "delivery_status", 10,
    [&received_msg, &received](const DeliveryStatus::SharedPtr msg) {
      received_msg = *msg;
      received = true;
        });

    auto tree = factory_.createTreeFromText(xml);
    auto status = tree.tickOnce();

    // 节点应返回 SUCCESS
    EXPECT_EQ(status, BT::NodeStatus::SUCCESS);

    // spin 一下让订阅回调执行
    rclcpp::spin_some(node_);

    // 验证消息内容
    ASSERT_TRUE(received) << "未收到 DeliveryStatus 消息";
    EXPECT_EQ(received_msg.order_id, "test_order");
    EXPECT_EQ(received_msg.state, DeliveryStatus::STATE_COMPLETE);
    EXPECT_EQ(received_msg.current_station, "station_C");
    EXPECT_FLOAT_EQ(received_msg.progress, 1.0f);
}

TEST_F(ReportDeliveryStatusTest, PublishesFailedState)
{
    const std::string xml =
    R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="Test">
        <ReportDeliveryStatus order_id="fail_order" state="6"
                              station_id="" progress="0.0" />
      </BehaviorTree>
    </root>
    )";

    auto tree = factory_.createTreeFromText(xml);
    auto status = tree.tickOnce();

    EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

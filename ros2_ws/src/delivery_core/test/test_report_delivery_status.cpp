/**
 * @file test_report_delivery_status.cpp
 * @brief ReportDeliveryStatus BT 节点单元测试。
 *
 * 验证 BT 叶节点是否正确将 XML 参数写入黑板输出端口，
 * 并在完成后返回 SUCCESS。
 * 注意：对外状态发布已由 delivery_manager 统一负责，
 * 本节点不再直接发布到 /delivery_status 话题。
 */

#include <gtest/gtest.h>

#include "behaviortree_cpp/bt_factory.h"
#include "delivery_core/nodes/report_delivery_status.hpp"
#include "delivery_interfaces/msg/delivery_status.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace delivery_core;
using DeliveryStatus = delivery_interfaces::msg::DeliveryStatus;

/**
 * @brief ReportDeliveryStatus 测试夹具。
 *
 * 夹具准备最小可用的 ROS 环境和 BT 工厂，
 * 让测试专注于"tick 之后黑板输出是否正确"。
 */
class ReportDeliveryStatusTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = rclcpp::Node::make_shared("test_report_node");

    factory_.registerBuilder<ReportDeliveryStatus>(
      "ReportDeliveryStatus",
      [this](const std::string & name, const BT::NodeConfig & config) {
        return std::make_unique<ReportDeliveryStatus>(name, config, node_);
      });
  }

  void TearDown() override
  {
    node_.reset();
    rclcpp::shutdown();
  }

  rclcpp::Node::SharedPtr node_;
  BT::BehaviorTreeFactory factory_;
};

/**
 * @brief 验证节点将状态正确写入黑板输出端口，并返回 SUCCESS。
 */
TEST_F(ReportDeliveryStatusTest, WritesBlackboardAndReturnsSuccess)
{
  const std::string xml =
    R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="Test">
        <ReportDeliveryStatus order_id="test_order" state="5"
                              station_id="station_C" progress="1.0"
                              bt_state="{bt_state}"
                              bt_station="{bt_station}"
                              bt_progress="{bt_progress}" />
      </BehaviorTree>
    </root>
    )";

  auto tree = factory_.createTreeFromText(xml);
  auto status = tree.tickOnce();

  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);

  // 验证黑板输出端口写入了正确的值。
  auto bb = tree.rootBlackboard();
  unsigned bt_state = 0;
  std::string bt_station;
  float bt_progress = 0.0f;

  EXPECT_TRUE(bb->get("bt_state", bt_state));
  EXPECT_TRUE(bb->get("bt_station", bt_station));
  EXPECT_TRUE(bb->get("bt_progress", bt_progress));

  EXPECT_EQ(bt_state, DeliveryStatus::STATE_COMPLETE);
  EXPECT_EQ(bt_station, "station_C");
  EXPECT_FLOAT_EQ(bt_progress, 1.0f);
}

/**
 * @brief 验证节点在失败态输入下仍能稳定返回 SUCCESS。
 */
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

/**
 * @file test_report_delivery_status.cpp
 * @brief ReportDeliveryStatus BT 节点单元测试。
 *
 * 这组测试只验证一个 BT 叶节点自己的职责：
 * 它是否会把 XML 里的状态参数正确转换成 ROS 消息并发布出去，
 * 以及在完成发布后是否返回 SUCCESS。
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
 * 夹具的职责是准备一个最小可用的 ROS 通信环境，
 * 让测试能在本进程内完成“发布一条消息，再把它接回来验证”的闭环。
 */
class ReportDeliveryStatusTest : public ::testing::Test
{
protected:
  /**
   * @brief 初始化测试节点、状态发布器和 BT builder。
   *
   * 这里不测试工厂装配逻辑本身，而是手动把待测节点注册进 factory，
   * 让每个测试都能专注于“tick 之后到底发了什么”。
   */
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = rclcpp::Node::make_shared("test_report_node");

    // 提前创建 publisher，作为待测节点构造时的外部依赖。
    status_pub_ = node_->create_publisher<DeliveryStatus>("delivery_status", 10);

    factory_.registerBuilder<ReportDeliveryStatus>(
      "ReportDeliveryStatus",
      [this](const std::string & name, const BT::NodeConfig & config) {
        return std::make_unique<ReportDeliveryStatus>(name, config, node_, status_pub_);
      });
  }

  /**
   * @brief 回收测试节点和 ROS 上下文。
   */
  void TearDown() override
  {
    node_.reset();
    rclcpp::shutdown();
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<DeliveryStatus>::SharedPtr status_pub_;
  BT::BehaviorTreeFactory factory_;
};

/**
 * @brief 验证节点会发布完整状态消息，并在 tick 后返回 SUCCESS。
 *
 * 这个用例覆盖最完整的一条 happy path。
 * 如果它失败，通常意味着：
 * - 节点没有真正发布消息
 * - 发布的字段值和 XML 不一致
 * - 节点执行后没有按约定返回 SUCCESS
 */
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

  // 在同一个节点上订阅同名 topic，把待测节点发布的消息回环接回来。
  DeliveryStatus received_msg;
  bool received = false;
  auto sub = node_->create_subscription<DeliveryStatus>(
    "delivery_status", 10,
    [&received_msg, &received](const DeliveryStatus::SharedPtr msg) {
      // 收到消息后立即保存副本，供断言逐字段核对。
      received_msg = *msg;
      received = true;
    });

  (void)sub;

  // 从 XML 生成一棵只包含待测叶节点的最小行为树。
  auto tree = factory_.createTreeFromText(xml);

  // tick 一次，触发节点读取 XML 参数并发布状态消息。
  auto status = tree.tickOnce();

  // 对这个节点来说，发布完成就算本次工作完成，因此应返回 SUCCESS。
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);

  // 仅仅发布并不保证订阅回调已经执行，所以还要显式 spin 一次。
  rclcpp::spin_some(node_);

  // 先确认消息确实到达了订阅端，否则后面逐字段比较都没有意义。
  ASSERT_TRUE(received) << "未收到 DeliveryStatus 消息";

  // 逐字段检查，确认 XML 输入被正确映射成了 ROS 消息。
  EXPECT_EQ(received_msg.order_id, "test_order");
  EXPECT_EQ(received_msg.state, DeliveryStatus::STATE_COMPLETE);
  EXPECT_EQ(received_msg.current_station, "station_C");
  EXPECT_FLOAT_EQ(received_msg.progress, 1.0f);
}

/**
 * @brief 验证节点在失败态输入下仍能稳定返回 SUCCESS。
 *
 * 这个用例防止实现只在“成功状态”下工作正常。
 * 它不重复验证字段映射，而是专注于另一类输入是否也能稳定跑通。
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

  // 换一组输入重新构树，确保节点不会因为状态变化而出现异常。
  auto tree = factory_.createTreeFromText(xml);
  auto status = tree.tickOnce();

  // 如果这里不是 SUCCESS，说明节点对另一条输入分支处理不稳。
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

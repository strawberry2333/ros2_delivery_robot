/**
 * @file test_check_battery.cpp
 * @brief CheckBattery BT 条件节点单元测试。
 *
 * 测试场景：
 * 1. 电量充足（>= threshold）→ SUCCESS
 * 2. 电量不足（< threshold）→ FAILURE
 * 3. 黑板无电量信息 → SUCCESS（默认充足）
 */

#include <gtest/gtest.h>

#include "behaviortree_cpp/bt_factory.h"
#include "delivery_core/nodes/check_battery.hpp"

using namespace delivery_core;

class CheckBatteryTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    factory_.registerNodeType<CheckBattery>("CheckBattery");
  }

  BT::BehaviorTreeFactory factory_;
};

TEST_F(CheckBatteryTest, BatterySufficient)
{
    // 电量 50% >= 阈值 20% → SUCCESS
    const std::string xml =
    R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="Test">
        <CheckBattery threshold="20.0" />
      </BehaviorTree>
    </root>
    )";

    auto tree = factory_.createTreeFromText(xml);
    tree.rootBlackboard()->set("battery_level", 50.0);

    auto status = tree.tickOnce();
    EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST_F(CheckBatteryTest, BatteryInsufficient)
{
    // 电量 10% < 阈值 20% → FAILURE
    const std::string xml =
    R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="Test">
        <CheckBattery threshold="20.0" />
      </BehaviorTree>
    </root>
    )";

    auto tree = factory_.createTreeFromText(xml);
    tree.rootBlackboard()->set("battery_level", 10.0);

    auto status = tree.tickOnce();
    EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(CheckBatteryTest, BatteryExactThreshold)
{
    // 电量 20% == 阈值 20% → SUCCESS（>=）
    const std::string xml =
    R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="Test">
        <CheckBattery threshold="20.0" />
      </BehaviorTree>
    </root>
    )";

    auto tree = factory_.createTreeFromText(xml);
    tree.rootBlackboard()->set("battery_level", 20.0);

    auto status = tree.tickOnce();
    EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST_F(CheckBatteryTest, NoBatteryInfoDefaultsSufficient)
{
    // 黑板中无 battery_level → SUCCESS
    const std::string xml =
    R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="Test">
        <CheckBattery threshold="20.0" />
      </BehaviorTree>
    </root>
    )";

    auto tree = factory_.createTreeFromText(xml);
    // 不设置 battery_level

    auto status = tree.tickOnce();
    EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

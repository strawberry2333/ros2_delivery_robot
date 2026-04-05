/**
 * @file test_check_battery.cpp
 * @brief CheckBattery BT 条件节点单元测试。
 *
 * 这组测试通过黑板直接给节点喂输入，专门验证“电量判断”这件事本身。
 * 这样一旦测试失败，就可以优先怀疑比较逻辑或默认分支，而不是外部 ROS 环境。
 */

#include <gtest/gtest.h>

#include "behaviortree_cpp/bt_factory.h"
#include "delivery_core/nodes/check_battery.hpp"

using namespace delivery_core;

/**
 * @brief CheckBattery 条件节点测试夹具。
 */
class CheckBatteryTest : public ::testing::Test
{
protected:
  /**
   * @brief 注册待测节点类型。
   *
   * CheckBattery 是纯条件节点，不依赖 ROS 节点或通信接口，
   * 因此夹具只需要准备一个 BT factory 即可。
   */
  void SetUp() override
  {
    factory_.registerNodeType<CheckBattery>("CheckBattery");
  }

  BT::BehaviorTreeFactory factory_;
};

/**
 * @brief 验证电量高于阈值时返回 SUCCESS。
 *
 * 这对应“任务可以继续执行”的正常路径。
 */
TEST_F(CheckBatteryTest, BatterySufficient)
{
  const std::string xml =
    R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="Test">
        <CheckBattery threshold="20.0" />
      </BehaviorTree>
    </root>
    )";

  auto tree = factory_.createTreeFromText(xml);

  // 用黑板模拟上游已经写入了当前电量。
  tree.rootBlackboard()->set("battery_level", 50.0);

  // 50 >= 20，预期应返回 SUCCESS。
  auto status = tree.tickOnce();
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

/**
 * @brief 验证电量低于阈值时返回 FAILURE。
 *
 * 这对应“任务应被拦下”的保护分支。
 */
TEST_F(CheckBatteryTest, BatteryInsufficient)
{
  const std::string xml =
    R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="Test">
        <CheckBattery threshold="20.0" />
      </BehaviorTree>
    </root>
    )";

  auto tree = factory_.createTreeFromText(xml);

  // 把电量压到阈值以下，模拟“继续执行已经不安全”的场景。
  tree.rootBlackboard()->set("battery_level", 10.0);

  auto status = tree.tickOnce();
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

/**
 * @brief 验证电量等于阈值时仍返回 SUCCESS。
 *
 * 这是边界值测试，用来钉住比较符号是 `>=` 而不是 `>`。
 */
TEST_F(CheckBatteryTest, BatteryExactThreshold)
{
  const std::string xml =
    R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="Test">
        <CheckBattery threshold="20.0" />
      </BehaviorTree>
    </root>
    )";

  auto tree = factory_.createTreeFromText(xml);

  // 精确命中阈值，防止后续修改把边界条件悄悄改坏。
  tree.rootBlackboard()->set("battery_level", 20.0);

  auto status = tree.tickOnce();
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

/**
 * @brief 验证缺少电量信息时走默认兜底分支。
 *
 * 这条测试防止节点在“上游暂时没写入电量”的情况下表现得过于脆弱。
 */
TEST_F(CheckBatteryTest, NoBatteryInfoDefaultsInsufficient)
{
  const std::string xml =
    R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="Test">
        <CheckBattery threshold="20.0" />
      </BehaviorTree>
    </root>
    )";

  auto tree = factory_.createTreeFromText(xml);

  // 故意不写 battery_level，缺少电量信息时应保守判定为电量不足。
  auto status = tree.tickOnce();
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

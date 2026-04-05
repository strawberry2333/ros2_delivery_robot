/**
 * @file test_bt_assembly.cpp
 * @brief BT XML 装配测试。
 *
 * 这组测试不执行真实配送流程，只验证“行为树描述文件能否被正确装配成树”。
 * 它主要防止 XML、节点注册代码和节点构造参数之间发生静默脱节。
 */

#include <gtest/gtest.h>

#include <fstream>
#include <string>

#include "behaviortree_cpp/bt_factory.h"
#include "delivery_core/nodes/check_battery.hpp"
#include "delivery_core/nodes/dock_at_station.hpp"
#include "delivery_core/nodes/navigate_to_station.hpp"
#include "delivery_core/nodes/report_delivery_status.hpp"
#include "delivery_core/nodes/wait_for_confirmation.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace delivery_core;
using DeliveryStatus = delivery_interfaces::msg::DeliveryStatus;

/**
 * @brief BT XML 装配测试夹具。
 *
 * 它负责准备 factory、最小 ROS 依赖，以及行为树里涉及到的自定义节点注册。
 * 如果后续 `createTreeFromFile()` 抛异常，排查方向就应集中在装配链路本身。
 */
class BtAssemblyTest : public ::testing::Test
{
protected:
  /**
   * @brief 初始化 factory 及其构造节点所需的最小环境。
   *
   * 这里不会真的发导航、发速度或等待确认；
   * 只要节点能被构造出来，这个测试的准备阶段就算完成。
   */
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = rclcpp::Node::make_shared("bt_assembly_test_node");

    // 先准备 BT 节点构造时依赖的 ROS 句柄。
    auto status_pub = node_->create_publisher<DeliveryStatus>("delivery_status", 10);
    auto cmd_vel_pub = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    auto nav_client = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      node_, "navigate_to_pose");

    // 逐个注册 XML 中会出现的自定义节点名。
    factory_.registerBuilder<NavigateToStation>(
      "NavigateToStation",
      [node = node_, nav_client](const std::string & name, const BT::NodeConfig & config) {
        return std::make_unique<NavigateToStation>(name, config, node, nav_client);
      });

    factory_.registerBuilder<DockAtStation>(
      "DockAtStation",
      [node = node_, cmd_vel_pub](const std::string & name, const BT::NodeConfig & config) {
        return std::make_unique<DockAtStation>(name, config, node, cmd_vel_pub);
      });

    factory_.registerBuilder<WaitForConfirmation>(
      "WaitForConfirmation",
      [node = node_, this](const std::string & name, const BT::NodeConfig & config) {
        return std::make_unique<WaitForConfirmation>(
          name, config, node, &load_flag_, &unload_flag_);
      });

    factory_.registerBuilder<ReportDeliveryStatus>(
      "ReportDeliveryStatus",
      [node = node_, status_pub](const std::string & name, const BT::NodeConfig & config) {
        return std::make_unique<ReportDeliveryStatus>(name, config, node, status_pub);
      });

    factory_.registerNodeType<CheckBattery>("CheckBattery");

    // 统一通过编译期宏拿到行为树目录，避免依赖当前工作目录。
    bt_dir_ = std::string(BEHAVIOR_TREES_DIR);
  }

  /**
   * @brief 释放测试过程中创建的 ROS 资源。
   */
  void TearDown() override
  {
    node_.reset();
    rclcpp::shutdown();
  }

  /**
   * @brief 判断指定 XML 文件是否存在。
   * @param[in] path 待检查的行为树文件路径。
   * @return 文件可读时返回 true，否则返回 false。
   */
  bool file_exists(const std::string & path)
  {
    std::ifstream f(path);
    return f.good();
  }

  rclcpp::Node::SharedPtr node_;
  BT::BehaviorTreeFactory factory_;
  std::atomic<bool> load_flag_{false};
  std::atomic<bool> unload_flag_{false};
  std::string bt_dir_;
};

/**
 * @brief 验证默认单单据行为树可以被成功装配。
 *
 * 如果这条测试失败，说明最基础的默认 BT 文件已经和当前节点注册代码不一致。
 */
TEST_F(BtAssemblyTest, SingleDeliveryXmlAssembles)
{
  const std::string path = bt_dir_ + "/single_delivery.xml";

  // 先确认文件真的存在，避免把“文件没了”误判成“XML 解析失败”。
  ASSERT_TRUE(file_exists(path)) << "找不到 " << path;

  EXPECT_NO_THROW({
    // 这里只创建树，不执行树，把问题范围锁定在“装配阶段”。
    auto tree = factory_.createTreeFromFile(path);
    (void)tree;
  });
}

/**
 * @brief 验证带重试逻辑的 robust 行为树可以被成功装配。
 *
 * 这棵树通常比基础版更复杂，因此更容易在重构后出现节点参数失配。
 */
TEST_F(BtAssemblyTest, SingleDeliveryRobustXmlAssembles)
{
  const std::string path = bt_dir_ + "/single_delivery_robust.xml";

  ASSERT_TRUE(file_exists(path)) << "找不到 " << path;

  EXPECT_NO_THROW({
    // 这里同样只验证“能不能装起来”，不验证重试行为本身。
    auto tree = factory_.createTreeFromFile(path);
    (void)tree;
  });
}

/**
 * @brief 验证完整任务树 delivery_mission.xml 可以被成功装配。
 *
 * 如果这条测试失败，说明更完整的任务级行为树已经无法被当前代码基础承接。
 */
TEST_F(BtAssemblyTest, DeliveryMissionXmlAssembles)
{
  const std::string path = bt_dir_ + "/delivery_mission.xml";

  ASSERT_TRUE(file_exists(path)) << "找不到 " << path;

  EXPECT_NO_THROW({
    // 仍然只验证解析、查找节点和实例化是否成功。
    auto tree = factory_.createTreeFromFile(path);
    (void)tree;
  });
}

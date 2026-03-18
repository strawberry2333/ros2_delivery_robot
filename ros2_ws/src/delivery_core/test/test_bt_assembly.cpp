/**
 * @file test_bt_assembly.cpp
 * @brief BT XML 装配测试。
 *
 * 验证所有 BT XML 能被正确解析和创建，不执行 BT，只验证注册和装配无报错。
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

class BtAssemblyTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = rclcpp::Node::make_shared("bt_assembly_test_node");

    auto status_pub = node_->create_publisher<DeliveryStatus>("delivery_status", 10);
    auto cmd_vel_pub = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    auto nav_client = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
            node_, "navigate_to_pose");

        // 注册所有 BT 节点
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

        // 获取 behavior_trees 目录路径
    bt_dir_ = std::string(BEHAVIOR_TREES_DIR);
  }

  void TearDown() override
  {
    node_.reset();
    rclcpp::shutdown();
  }

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

TEST_F(BtAssemblyTest, SingleDeliveryXmlAssembles)
{
    const std::string path = bt_dir_ + "/single_delivery.xml";
    ASSERT_TRUE(file_exists(path)) << "找不到 " << path;
    EXPECT_NO_THROW({
    auto tree = factory_.createTreeFromFile(path);
    });
}

TEST_F(BtAssemblyTest, SingleDeliveryRobustXmlAssembles)
{
    const std::string path = bt_dir_ + "/single_delivery_robust.xml";
    ASSERT_TRUE(file_exists(path)) << "找不到 " << path;
    EXPECT_NO_THROW({
    auto tree = factory_.createTreeFromFile(path);
    });
}

TEST_F(BtAssemblyTest, DeliveryMissionXmlAssembles)
{
    const std::string path = bt_dir_ + "/delivery_mission.xml";
    ASSERT_TRUE(file_exists(path)) << "找不到 " << path;
    EXPECT_NO_THROW({
    auto tree = factory_.createTreeFromFile(path);
    });
}

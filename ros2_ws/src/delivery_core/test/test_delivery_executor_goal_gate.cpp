#include <gtest/gtest.h>

#include <memory>
#include <string>

#include "delivery_core/delivery_executor.hpp"
#include "delivery_interfaces/action/execute_delivery.hpp"
#include "rclcpp/rclcpp.hpp"

namespace delivery_core
{

class DeliveryExecutorTestAccess
{
public:
  static void add_station(
    DeliveryExecutor & executor,
    const std::string & station_id,
    uint8_t station_type)
  {
    Station station;
    station.id = station_id;
    station.type = station_type;
    executor.stations_[station_id] = station;
  }

  static rclcpp_action::GoalResponse handle_goal(
    DeliveryExecutor & executor,
    std::shared_ptr<const DeliveryExecutor::ExecuteDelivery::Goal> goal)
  {
    const rclcpp_action::GoalUUID uuid{};
    return executor.handle_goal(uuid, goal);
  }

  static bool goal_inflight(const DeliveryExecutor & executor)
  {
    return executor.goal_inflight_.load(std::memory_order_acquire);
  }

  static void clear_goal_inflight(DeliveryExecutor & executor)
  {
    executor.goal_inflight_.store(false, std::memory_order_release);
  }
};

namespace
{

class DeliveryExecutorGoalGateTest : public ::testing::Test
{
protected:
  using ExecuteDelivery = delivery_interfaces::action::ExecuteDelivery;

  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    executor_ = std::make_unique<DeliveryExecutor>();
    DeliveryExecutorTestAccess::add_station(*executor_, "station_A", 0);
    DeliveryExecutorTestAccess::add_station(*executor_, "station_C", 1);
  }

  void TearDown() override
  {
    executor_.reset();
    rclcpp::shutdown();
  }

  std::shared_ptr<const ExecuteDelivery::Goal> make_goal(const std::string & order_id) const
  {
    auto goal = std::make_shared<ExecuteDelivery::Goal>();
    goal->order.order_id = order_id;
    goal->order.pickup_station = "station_A";
    goal->order.dropoff_station = "station_C";
    return goal;
  }

  std::unique_ptr<DeliveryExecutor> executor_;
};

TEST_F(DeliveryExecutorGoalGateTest, RejectsSecondGoalBeforeExecutionThreadStarts)
{
  EXPECT_EQ(
    DeliveryExecutorTestAccess::handle_goal(*executor_, make_goal("order_1")),
    rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE);
  EXPECT_TRUE(DeliveryExecutorTestAccess::goal_inflight(*executor_));

  EXPECT_EQ(
    DeliveryExecutorTestAccess::handle_goal(*executor_, make_goal("order_2")),
    rclcpp_action::GoalResponse::REJECT);
}

TEST_F(DeliveryExecutorGoalGateTest, AcceptsNewGoalAfterInflightFlagCleared)
{
  EXPECT_EQ(
    DeliveryExecutorTestAccess::handle_goal(*executor_, make_goal("order_1")),
    rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE);

  DeliveryExecutorTestAccess::clear_goal_inflight(*executor_);

  EXPECT_EQ(
    DeliveryExecutorTestAccess::handle_goal(*executor_, make_goal("order_2")),
    rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE);
}

}  // namespace

}  // namespace delivery_core

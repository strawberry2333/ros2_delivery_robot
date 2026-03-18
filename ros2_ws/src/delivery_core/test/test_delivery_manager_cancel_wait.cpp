#include <gtest/gtest.h>

#include <chrono>
#include <future>
#include <memory>
#include <thread>

#include "delivery_core/delivery_manager.hpp"
#include "delivery_interfaces/action/execute_delivery.hpp"
#include "rclcpp/rclcpp.hpp"

namespace delivery_core
{

class DeliveryManagerTestAccess
{
public:
  using WrappedResult = DeliveryManager::ExecuteDeliveryGoalHandle::WrappedResult;

  static void set_cancel_completion_wait_timeout(
    DeliveryManager & manager,
    double timeout_sec)
  {
    manager.cancel_completion_wait_timeout_sec_ = timeout_sec;
  }

  static bool wait_for_terminal_result_after_cancel(
    DeliveryManager & manager,
    const std::string & order_id,
    const std::shared_future<WrappedResult> & result_future)
  {
    return manager.wait_for_terminal_result_after_cancel(order_id, result_future);
  }
};

namespace
{

class DeliveryManagerCancelWaitTest : public ::testing::Test
{
protected:
  using WrappedResult = DeliveryManagerTestAccess::WrappedResult;

  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions options;
    options.parameter_overrides({
          {"use_sim_time", false},
          {"publish_initial_pose", false},
        });
    manager_ = std::make_unique<DeliveryManager>(options);
  }

  void TearDown() override
  {
    manager_.reset();
    rclcpp::shutdown();
  }

  WrappedResult make_canceled_result() const
  {
    WrappedResult wrapped_result;
    wrapped_result.code = rclcpp_action::ResultCode::CANCELED;
    wrapped_result.result =
      std::make_shared<delivery_interfaces::action::ExecuteDelivery::Result>();
    return wrapped_result;
  }

  std::unique_ptr<DeliveryManager> manager_;
};

TEST_F(DeliveryManagerCancelWaitTest, WaitsForTerminalResultWithinGraceWindow)
{
  DeliveryManagerTestAccess::set_cancel_completion_wait_timeout(*manager_, 0.2);

  std::promise<WrappedResult> promise;
  auto future = promise.get_future().share();
  auto producer = std::thread([promise = std::move(promise), this]() mutable {
        std::this_thread::sleep_for(std::chrono::milliseconds(80));
        promise.set_value(make_canceled_result());
      });

  const auto start = std::chrono::steady_clock::now();
  EXPECT_TRUE(DeliveryManagerTestAccess::wait_for_terminal_result_after_cancel(
      *manager_, "order_timeout", future));
  const auto elapsed = std::chrono::steady_clock::now() - start;

  producer.join();
  EXPECT_GE(elapsed, std::chrono::milliseconds(70));
}

TEST_F(DeliveryManagerCancelWaitTest, ReturnsFalseWhenExecutorDoesNotFinishInTime)
{
  DeliveryManagerTestAccess::set_cancel_completion_wait_timeout(*manager_, 0.05);

  std::promise<WrappedResult> promise;
  auto future = promise.get_future().share();

  const auto start = std::chrono::steady_clock::now();
  EXPECT_FALSE(DeliveryManagerTestAccess::wait_for_terminal_result_after_cancel(
      *manager_, "order_timeout", future));
  const auto elapsed = std::chrono::steady_clock::now() - start;

  EXPECT_GE(elapsed, std::chrono::milliseconds(40));
}

}  // namespace

}  // namespace delivery_core

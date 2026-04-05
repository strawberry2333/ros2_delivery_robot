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

/**
 * @brief DeliveryManager 取消等待逻辑的白盒测试访问器。
 *
 * 被测逻辑是内部实现细节，不适合为了测试而变成正式 public API。
 * 因此这里只暴露测试真正需要的两个入口。
 */
class DeliveryManagerTestAccess
{
public:
  using WrappedResult = DeliveryManager::ExecuteDeliveryGoalHandle::WrappedResult;

  /**
   * @brief 调整取消后等待终态结果的超时时间。
   * @param[in,out] manager 被测 manager。
   * @param[in] timeout_sec 等待超时，单位秒。
   */
  static void set_cancel_completion_wait_timeout(
    DeliveryManager & manager,
    double timeout_sec)
  {
    manager.cancel_completion_wait_timeout_sec_ = timeout_sec;
  }

  /**
   * @brief 调用内部等待函数。
   * @param[in,out] manager 被测 manager。
   * @param[in] order_id 当前订单 ID。
   * @param[in] result_future 下游 executor 的终态结果 future。
   * @return 在等待窗口内收到终态结果时返回 true，否则返回 false。
   */
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

/**
 * @brief 取消后等待终态结果逻辑测试夹具。
 *
 * 这组测试不启动真实 action server，只验证“等多久、在什么条件下返回 true/false”。
 */
class DeliveryManagerCancelWaitTest : public ::testing::Test
{
protected:
  using WrappedResult = DeliveryManagerTestAccess::WrappedResult;

  /**
   * @brief 创建最小可测的 DeliveryManager 实例。
   */
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

  /**
   * @brief 销毁 manager 并关闭 ROS。
   */
  void TearDown() override
  {
    manager_.reset();
    rclcpp::shutdown();
  }

  /**
   * @brief 构造一个“已取消”的最小终态结果。
   * @return `ResultCode::CANCELED` 对应的 WrappedResult。
   */
  WrappedResult make_canceled_result() const
  {
    WrappedResult wrapped_result;

    // 明确标记这是 cancel 成功收尾后的终态。
    wrapped_result.code = rclcpp_action::ResultCode::CANCELED;
    wrapped_result.result =
      std::make_shared<delivery_interfaces::action::ExecuteDelivery::Result>();
    return wrapped_result;
  }

  std::unique_ptr<DeliveryManager> manager_;
};

/**
 * @brief 验证终态结果在宽限时间内到达时返回 true。
 *
 * 这条测试防止 manager 取消后过早放弃等待，导致下游已经完成的终态信息被丢掉。
 */
TEST_F(DeliveryManagerCancelWaitTest, WaitsForTerminalResultWithinGraceWindow)
{
  // 等待窗口设得比结果到达时间更长，预期应等到结果。
  DeliveryManagerTestAccess::set_cancel_completion_wait_timeout(*manager_, 0.2);

  std::promise<WrappedResult> promise;
  auto future = promise.get_future().share();

  auto producer = std::thread([promise = std::move(promise), this]() mutable {
        // 先延迟一小会儿，模拟 executor 取消后还在做收尾。
        std::this_thread::sleep_for(std::chrono::milliseconds(80));

        // 然后补回一个终态结果，表示下游已经完成。
        promise.set_value(make_canceled_result());
      });

  const auto start = std::chrono::steady_clock::now();
  EXPECT_TRUE(DeliveryManagerTestAccess::wait_for_terminal_result_after_cancel(
      *manager_, "order_timeout", future));
  const auto elapsed = std::chrono::steady_clock::now() - start;

  producer.join();

  // 确认函数不是“秒返回”，而是真的等待到了接近结果到达时间。
  EXPECT_GE(elapsed, std::chrono::milliseconds(70));
}

/**
 * @brief 验证终态结果迟迟不到时，等待函数会超时返回 false。
 *
 * 这条测试防止取消流程无限阻塞。
 */
TEST_F(DeliveryManagerCancelWaitTest, ReturnsFalseWhenExecutorDoesNotFinishInTime)
{
  // 把等待窗口压短，确保覆盖超时分支。
  DeliveryManagerTestAccess::set_cancel_completion_wait_timeout(*manager_, 0.05);

  std::promise<WrappedResult> promise;
  auto future = promise.get_future().share();

  const auto start = std::chrono::steady_clock::now();
  EXPECT_FALSE(DeliveryManagerTestAccess::wait_for_terminal_result_after_cancel(
      *manager_, "order_timeout", future));
  const auto elapsed = std::chrono::steady_clock::now() - start;

  // 再确认它确实等了一段时间，而不是立即返回。
  EXPECT_GE(elapsed, std::chrono::milliseconds(40));
}

}  // namespace

}  // namespace delivery_core

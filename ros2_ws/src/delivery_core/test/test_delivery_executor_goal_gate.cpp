#include <gtest/gtest.h>

#include <memory>
#include <string>

#include "delivery_core/delivery_executor.hpp"
#include "delivery_interfaces/action/execute_delivery.hpp"
#include "rclcpp/rclcpp.hpp"

namespace delivery_core
{

/**
 * @brief DeliveryExecutor 白盒测试访问器。
 *
 * 这些测试需要直接碰 `DeliveryExecutor` 的少量内部状态，才能把
 * “接单门禁逻辑”从完整 action server 异步流程里拆出来单独验证。
 * 为了不污染生产 API，这里只暴露测试真正需要的最小入口。
 */
class DeliveryExecutorTestAccess
{
public:
  /**
   * @brief 向 executor 手动注入一个站点定义。
   * @param[in,out] executor 被测执行器实例。
   * @param[in] station_id 站点 ID。
   * @param[in] station_type 站点类型。
   *
   * 这样做的目的不是测试站点加载，而是提前满足 goal 校验依赖，
   * 让后续测试只聚焦在 gate 逻辑。
   */
  static void add_station(
    DeliveryExecutor & executor,
    const std::string & station_id,
    uint8_t station_type)
  {
    // 手动塞入最小站点集合，让输入校验先过关。
    Station station;
    station.id = station_id;
    station.type = station_type;
    executor.stations_[station_id] = station;
  }

  /**
   * @brief 直接调用内部 goal 处理函数。
   * @param[in,out] executor 被测执行器实例。
   * @param[in] goal 待提交的配送 goal。
   * @return 该 goal 当前应得到的接收结果。
   *
   * 这能绕开 action server 的网络和线程外壳，把测试问题收敛成：
   * “当前内部状态下，这单究竟该接还是该拒绝？”
   */
  static rclcpp_action::GoalResponse handle_goal(
    DeliveryExecutor & executor,
    std::shared_ptr<const DeliveryExecutor::ExecuteDelivery::Goal> goal)
  {
    // UUID 在这个测试里并不是关注点，因此给一个默认值即可。
    const rclcpp_action::GoalUUID uuid{};
    return executor.handle_goal(uuid, goal);
  }

  /**
   * @brief 读取当前是否已有执行中的 goal。
   * @param[in] executor 被测执行器实例。
   * @return `goal_inflight_` 的当前值。
   */
  static bool goal_inflight(const DeliveryExecutor & executor)
  {
    // 直接读原子标志，确认第一个 goal 接收后是否真的关上了门。
    return executor.goal_inflight_.load(std::memory_order_acquire);
  }

  /**
   * @brief 手动清除执行中标志。
   * @param[in,out] executor 被测执行器实例。
   *
   * 这一步模拟的是“前一单已经执行完，执行线程把门重新打开”。
   */
  static void clear_goal_inflight(DeliveryExecutor & executor)
  {
    // 直接释放互斥状态，验证 gate 是否能重新接收下一单。
    executor.goal_inflight_.store(false, std::memory_order_release);
  }
};

namespace
{

/**
 * @brief DeliveryExecutor 接单门禁测试夹具。
 *
 * 这组测试不验证 BT 执行本身，而是只验证入口 gate：
 * 同一时间是否只允许一张订单进入执行器。
 */
class DeliveryExecutorGoalGateTest : public ::testing::Test
{
protected:
  using ExecuteDelivery = delivery_interfaces::action::ExecuteDelivery;

  /**
   * @brief 创建一个最小可测的 DeliveryExecutor。
   *
   * 这里不启动完整生命周期，也不创建真实 action server；
   * 只准备好能让 `handle_goal()` 完成判断所需的最小状态。
   */
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    executor_ = std::make_unique<DeliveryExecutor>();

    // 只注入一对合法站点，避免测试被“站点不存在/类型不合法”干扰。
    DeliveryExecutorTestAccess::add_station(*executor_, "station_A", 0);
    DeliveryExecutorTestAccess::add_station(*executor_, "station_C", 1);
  }

  /**
   * @brief 销毁被测对象并关闭 ROS。
   */
  void TearDown() override
  {
    executor_.reset();
    rclcpp::shutdown();
  }

  /**
   * @brief 生成一份合法的配送 goal。
   * @param[in] order_id 本次测试使用的订单 ID。
   * @return pickup 和 dropoff 都合法的 goal 对象。
   *
   * 统一复用合法输入，可以让失败更明确地归因于 gate，而不是输入参数错误。
   */
  std::shared_ptr<const ExecuteDelivery::Goal> make_goal(const std::string & order_id) const
  {
    auto goal = std::make_shared<ExecuteDelivery::Goal>();

    // 只让 order_id 变化，其余字段保持合法且稳定。
    goal->order.order_id = order_id;
    goal->order.pickup_station = "station_A";
    goal->order.dropoff_station = "station_C";
    return goal;
  }

  std::unique_ptr<DeliveryExecutor> executor_;
};

/**
 * @brief 验证已有执行中 goal 时，新的 goal 会被拒绝。
 *
 * 测试意图：
 * - 第一个 goal 被接收后，应立刻把执行器置为“忙碌中”
 * - 在它结束前，第二个 goal 不能再被放进来
 *
 * 如果这里失败，说明执行器可能会并发处理多单，订单状态和资源占用都会混乱。
 */
TEST_F(DeliveryExecutorGoalGateTest, RejectsSecondGoalBeforeExecutionThreadStarts)
{
  // 第一个 goal 应该被正常接收，并把 gate 关上。
  EXPECT_EQ(
    DeliveryExecutorTestAccess::handle_goal(*executor_, make_goal("order_1")),
    rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE);

  // 明确确认 inflight 标志已经被置位，说明“忙碌中”状态确实建立起来了。
  EXPECT_TRUE(DeliveryExecutorTestAccess::goal_inflight(*executor_));

  // 第二个 goal 紧接着到达，模拟最容易出竞态的真实场景。
  EXPECT_EQ(
    DeliveryExecutorTestAccess::handle_goal(*executor_, make_goal("order_2")),
    rclcpp_action::GoalResponse::REJECT);
}

/**
 * @brief 验证执行结束后，新的 goal 可以再次被接收。
 *
 * 测试意图：
 * - gate 不仅要会“拒绝并发”
 * - 也要会在任务结束后“重新打开”
 *
 * 如果这里失败，说明执行器可能在完成一单后永久卡死在忙碌状态。
 */
TEST_F(DeliveryExecutorGoalGateTest, AcceptsNewGoalAfterInflightFlagCleared)
{
  // 先制造一个“已进入执行中”的状态。
  EXPECT_EQ(
    DeliveryExecutorTestAccess::handle_goal(*executor_, make_goal("order_1")),
    rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE);

  // 然后手动模拟执行线程收尾，把门重新打开。
  DeliveryExecutorTestAccess::clear_goal_inflight(*executor_);

  // 此时新的 goal 应该恢复可接收。
  EXPECT_EQ(
    DeliveryExecutorTestAccess::handle_goal(*executor_, make_goal("order_2")),
    rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE);
}

}  // namespace

}  // namespace delivery_core

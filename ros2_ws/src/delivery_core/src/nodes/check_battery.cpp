/**
 * @file check_battery.cpp
 * @brief CheckBattery 条件节点实现。
 *
 * 这个节点通常放在配送任务最前面，作为"是否值得继续执行"的门禁。
 * 它只做判断，不做控制，不发消息，不修改电量状态。
 *
 * 业务语义：
 * - `SUCCESS`：电量满足阈值，允许继续执行配送任务
 * - `FAILURE`：电量不足，当前任务应终止或进入其它恢复逻辑
 */

#include "delivery_core/nodes/check_battery.hpp"
#include "rclcpp/rclcpp.hpp"

namespace delivery_core
{

CheckBattery::CheckBattery(
  const std::string & name, const BT::NodeConfig & config)
: BT::ConditionNode(name, config)
{
  // 条件节点不需要在构造阶段做额外初始化，所有数据都在 tick() 时读取。
}

BT::NodeStatus CheckBattery::tick()
{
  // 阈值可由行为树 XML 覆盖，默认值用于没有显式配置时的兜底。
  double threshold = 20.0;
  getInput("threshold", threshold);

  // 从黑板读取当前电量。这里的电量通常由上层任务或模拟器维护。
  double battery_level = 0.0;
  auto bb = config().blackboard;
  if (!bb->get("battery_level", battery_level)) {
    RCLCPP_WARN(rclcpp::get_logger("CheckBattery"),
      "CheckBattery: blackboard missing battery_level, defaulting to insufficient");
    return BT::NodeStatus::FAILURE;
  }

  // 电量达到阈值则允许继续；否则直接失败，把决策交回父树处理。
  if (battery_level >= threshold) {
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace delivery_core

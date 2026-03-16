/**
 * @file check_battery.cpp
 * @brief CheckBattery 条件节点实现。
 *
 * 从黑板读取 battery_level 浮点值（0.0~100.0），
 * 与 threshold 端口比较，>=threshold 返回 SUCCESS，否则 FAILURE。
 */

#include "delivery_core/nodes/check_battery.hpp"

namespace delivery_core
{

CheckBattery::CheckBattery(
    const std::string & name, const BT::NodeConfig & config)
    : BT::ConditionNode(name, config)
{
}

BT::NodeStatus CheckBattery::tick()
{
    double threshold = 20.0;
    getInput("threshold", threshold);

    double battery_level = 0.0;
    auto bb = config().blackboard;
    if (!bb->get("battery_level", battery_level))
    {
        // 黑板中无电量信息，默认充足
        return BT::NodeStatus::SUCCESS;
    }

    if (battery_level >= threshold)
    {
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        return BT::NodeStatus::FAILURE;
    }
}

}  // namespace delivery_core

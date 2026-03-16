#pragma once

/**
 * @file check_battery.hpp
 * @brief BT 条件节点：检查电池电量是否充足。
 *
 * 从黑板读取 battery_level，与 threshold 端口比较。
 * battery_level >= threshold 返回 SUCCESS，否则 FAILURE。
 */

#include "behaviortree_cpp/condition_node.h"

namespace delivery_core
{

class CheckBattery : public BT::ConditionNode
{
public:
    CheckBattery(const std::string & name, const BT::NodeConfig & config);

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("threshold", 20.0, "电量阈值（百分比）"),
        };
    }

    BT::NodeStatus tick() override;
};

}  // namespace delivery_core

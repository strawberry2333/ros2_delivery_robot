#pragma once

/**
 * @file check_battery.hpp
 * @brief BT 条件节点：检查电池电量是否充足。
 *
 * 这个节点通常放在单次配送任务的前置检查位置，用来决定当前订单是否
 * 还能继续执行。它不负责充电，也不负责恢复电量，只负责读取黑板中的
 * `battery_level` 并和阈值比较。
 *
 * 业务语义：
 * - `SUCCESS`：电量足够，可以继续执行后续导航/装卸确认流程
 * - `FAILURE`：电量不足，当前任务应中止或切换到其它恢复逻辑
 *
 * 这里采用条件节点而不是动作节点，是因为它没有外部副作用，只做快速判定。
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
      // 阈值由行为树 XML 或父节点下发，便于不同任务配置不同电量门槛。
      BT::InputPort<double>("threshold", 20.0, "电量阈值（百分比）"),
    };
  }

  /**
   * @brief 读取黑板中的电量并做阈值判定。
   *
   * 这个函数应尽量保持轻量：它只读取状态，不等待、不阻塞、不修改外部
   * 资源。行为树每次 tick 到这里时，都是在判断“当前任务是否允许继续”。
   */
  BT::NodeStatus tick() override;
};

}  // namespace delivery_core

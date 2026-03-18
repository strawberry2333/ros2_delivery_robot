#pragma once

/**
 * @file wait_for_confirmation.hpp
 * @brief BT 叶节点：等待人工装/卸货确认。
 *
 * 通过注入的 atomic<bool> 指针检查确认信号，
 * 确认服务由 delivery_executor 持有。
 *
 * @note 当前为 demo 实现：全局 atomic 标志轮询，不与订单 ID 绑定
 */

#include <atomic>
#include <chrono>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"

namespace delivery_core
{

/**
 * @brief 等待人工确认的 BT 叶节点（StatefulActionNode）
 *
 * 输入端口：
 * - confirm_type: "load" 或 "unload"
 * - timeout_sec: 超时秒数，默认 60.0
 */
class WaitForConfirmation : public BT::StatefulActionNode
{
public:
  WaitForConfirmation(
    const std::string & name,
    const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node,
    std::atomic<bool> * load_flag,
    std::atomic<bool> * unload_flag);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("confirm_type", "load 或 unload"),
      BT::InputPort<double>("timeout_sec", 60.0, "超时秒数"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::atomic<bool> * load_flag_;
  std::atomic<bool> * unload_flag_;

  std::chrono::steady_clock::time_point start_time_;
  double timeout_sec_{60.0};
  std::string confirm_type_;
};

}  // namespace delivery_core

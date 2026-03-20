#pragma once

/**
 * @file wait_for_confirmation.hpp
 * @brief BT 叶节点：等待人工装/卸货确认。
 *
 * 这是配送流程里的人工交互门槛节点，通常位于到达取货点后的“等待装货”
 * 阶段，以及到达送货点后的“等待卸货”阶段。它通过轮询外部注入的
 * `atomic<bool>` 标志，等待人工确认信号到来。
 *
 * 业务语义：
 * - `RUNNING`：正在等待确认，行为树暂停在该阶段
 * - `SUCCESS`：对应确认信号已收到，可以进入下一阶段
 * - `FAILURE`：等待超时，当前订单应判定失败或终止
 *
 * 确认服务由 `delivery_executor` 持有，这个节点只消费确认标志，不直接创建服务。
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
 * - `confirm_type`：`"load"` 或 `"unload"`，决定等待哪一种人工确认
 * - `timeout_sec`：超时秒数，默认 60.0
 *
 * 这个节点是 StatefulActionNode，因为等待过程是持续性的：
 * 进入 `onStart()` 后开始计时，随后在 `onRunning()` 中反复检查确认标志。
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
      // confirm_type 决定节点监听哪一个确认标志，也决定日志里显示“装货/卸货”。
      BT::InputPort<std::string>("confirm_type", "load 或 unload"),
      // 超时由行为树 XML 配置，便于不同任务给不同等待窗口。
      BT::InputPort<double>("timeout_sec", 60.0, "超时秒数"),
    };
  }

  /**
   * @brief 读取端口参数，重置对应确认标志，并启动计时。
   *
   * 这里的核心工作是把节点切换到“等待态”：
   * - 清掉旧的确认标志，避免上一单残留状态影响当前单
   * - 记录开始时间，用于后续超时判断
   * - 输出清晰日志，说明当前等待的是装货还是卸货
   */
  BT::NodeStatus onStart() override;

  /**
   * @brief 轮询确认标志并检查是否超时。
   *
   * 如果确认标志被外部服务置位，则返回 SUCCESS；
   * 如果超时，则返回 FAILURE；
   * 否则继续返回 RUNNING，维持等待状态。
   */
  BT::NodeStatus onRunning() override;

  /**
   * @brief 节点被行为树中断时的收尾入口。
   *
   * 当前实现只是记录日志，因为确认标志本身不需要特殊释放。
   */
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

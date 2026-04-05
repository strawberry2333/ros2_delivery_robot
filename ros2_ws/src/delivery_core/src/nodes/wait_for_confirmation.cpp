/**
 * @file wait_for_confirmation.cpp
 * @brief WaitForConfirmation BT 叶节点实现。
 *
 * 这个节点把“等待人工装货/卸货确认”变成行为树里的一个显式阶段。
 * 它本身不创建服务，也不与订单 ID 绑定，只轮询注入的确认标志。
 *
 * 业务语义：
 * - 确认标志被置位：返回 SUCCESS，表示可以进入下一阶段
 * - 等待期间：返回 RUNNING，维持当前停靠状态
 * - 超时：返回 FAILURE，表示当前订单未按时完成人工确认
 */

#include "delivery_core/nodes/wait_for_confirmation.hpp"

namespace delivery_core
{

WaitForConfirmation::WaitForConfirmation(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node,
  std::atomic<bool> * load_flag,
  std::atomic<bool> * unload_flag)
: BT::StatefulActionNode(name, config),
  node_(node),
  load_flag_(load_flag),
  unload_flag_(unload_flag)
{
  // 构造阶段只保存确认标志指针，真实等待逻辑在 onStart()/onRunning() 中。
}

BT::NodeStatus WaitForConfirmation::onStart()
{
  // 读取端口参数，确认当前节点要等待的是装货还是卸货。
  if (!getInput("confirm_type", confirm_type_)) {
    RCLCPP_ERROR(node_->get_logger(), "WaitForConfirmation: 缺少 confirm_type");
    return BT::NodeStatus::FAILURE;
  }

  // confirm_type 必须为 "load" 或 "unload"，拒绝拼写错误静默降级到 unload 分支。
  if (confirm_type_ != "load" && confirm_type_ != "unload") {
    RCLCPP_ERROR(node_->get_logger(),
      "WaitForConfirmation: confirm_type 非法 [%s]，必须为 \"load\" 或 \"unload\"",
      confirm_type_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  // 超时时长由 XML 配置，默认 60 秒。
  getInput("timeout_sec", timeout_sec_);

  // 进入等待态前先清掉旧标志，避免上一单的确认残留到当前单。
  std::atomic<bool> & flag =
    (confirm_type_ == "load") ? *load_flag_ : *unload_flag_;
  flag.store(false, std::memory_order_release);

  // 记录开始等待的时间点，用于后续超时判断。
  start_time_ = std::chrono::steady_clock::now();

  // 便于从日志里区分当前到底是在等装货还是卸货。
  const std::string label = (confirm_type_ == "load") ? "装货" : "卸货";
  const std::string service = (confirm_type_ == "load") ? "confirm_load" : "confirm_unload";

  RCLCPP_INFO(node_->get_logger(),
        "WaitForConfirmation: 等待%s确认 (超时 %.0f 秒, 调用 /%s)",
        label.c_str(), timeout_sec_, service.c_str());

  // 进入持续等待状态，后续由 onRunning() 反复检查标志和时间。
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForConfirmation::onRunning()
{
  // 根据当前 confirm_type 选择对应的确认标志。
  std::atomic<bool> & flag =
    (confirm_type_ == "load") ? *load_flag_ : *unload_flag_;

  // 收到人工确认后，当前阶段结束，可以继续后续动作。
  if (flag.load(std::memory_order_acquire)) {
    const std::string label = (confirm_type_ == "load") ? "装货" : "卸货";
    RCLCPP_INFO(node_->get_logger(), "WaitForConfirmation: %s确认完成", label.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  // 如果还没收到确认，就检查是否已经等待超时。
  const auto elapsed = std::chrono::steady_clock::now() - start_time_;
  if (std::chrono::duration_cast<std::chrono::duration<double>>(elapsed).count() >
    timeout_sec_)
  {
    const std::string label = (confirm_type_ == "load") ? "装货" : "卸货";
    RCLCPP_WARN(node_->get_logger(),
            "WaitForConfirmation: %s确认超时 (%.0f 秒)", label.c_str(), timeout_sec_);
    return BT::NodeStatus::FAILURE;
  }

  // 既没收到确认，也没超时，继续保持等待。
  return BT::NodeStatus::RUNNING;
}

void WaitForConfirmation::onHalted()
{
  // 当前实现只做轮询，不持有外部资源，因此取消时只需要记录日志。
  RCLCPP_INFO(node_->get_logger(), "WaitForConfirmation: 等待被取消");
}

}  // namespace delivery_core

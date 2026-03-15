/**
 * @file wait_for_confirmation.cpp
 * @brief WaitForConfirmation BT 叶节点实现。
 *
 * 轮询检查 atomic<bool> 标志，等待人工确认信号或超时。
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
}

BT::NodeStatus WaitForConfirmation::onStart()
{
    // 读取端口参数
    if (!getInput("confirm_type", confirm_type_))
    {
        RCLCPP_ERROR(node_->get_logger(), "WaitForConfirmation: 缺少 confirm_type");
        return BT::NodeStatus::FAILURE;
    }

    getInput("timeout_sec", timeout_sec_);

    // 重置对应的确认标志
    std::atomic<bool> & flag =
        (confirm_type_ == "load") ? *load_flag_ : *unload_flag_;
    flag.store(false, std::memory_order_release);

    start_time_ = std::chrono::steady_clock::now();

    const std::string label = (confirm_type_ == "load") ? "装货" : "卸货";
    const std::string service = (confirm_type_ == "load") ? "confirm_load" : "confirm_unload";

    RCLCPP_INFO(node_->get_logger(),
        "WaitForConfirmation: 等待%s确认 (超时 %.0f 秒, 调用 /%s)",
        label.c_str(), timeout_sec_, service.c_str());

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForConfirmation::onRunning()
{
    std::atomic<bool> & flag =
        (confirm_type_ == "load") ? *load_flag_ : *unload_flag_;

    // 检查确认信号
    if (flag.load(std::memory_order_acquire))
    {
        const std::string label = (confirm_type_ == "load") ? "装货" : "卸货";
        RCLCPP_INFO(node_->get_logger(), "WaitForConfirmation: %s确认完成", label.c_str());
        return BT::NodeStatus::SUCCESS;
    }

    // 检查超时
    const auto elapsed = std::chrono::steady_clock::now() - start_time_;
    if (std::chrono::duration_cast<std::chrono::duration<double>>(elapsed).count()
        > timeout_sec_)
    {
        const std::string label = (confirm_type_ == "load") ? "装货" : "卸货";
        RCLCPP_WARN(node_->get_logger(),
            "WaitForConfirmation: %s确认超时 (%.0f 秒)", label.c_str(), timeout_sec_);
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
}

void WaitForConfirmation::onHalted()
{
    // 被取消时无需特殊清理
    RCLCPP_INFO(node_->get_logger(), "WaitForConfirmation: 等待被取消");
}

}  // namespace delivery_core

/**
 * @file navigate_to_station.cpp
 * @brief NavigateToStation BT 叶节点实现。
 */

#include "delivery_core/nodes/navigate_to_station.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

namespace delivery_core
{

NavigateToStation::NavigateToStation(
    const std::string & name,
    const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node,
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client)
    : BT::StatefulActionNode(name, config),
      node_(node),
      nav_client_(nav_client)
{
}

BT::NodeStatus NavigateToStation::onStart()
{
    // 获取目标站点 ID
    std::string station_id;
    if (!getInput("station_id", station_id))
    {
        RCLCPP_ERROR(node_->get_logger(), "NavigateToStation: 缺少 station_id 端口");
        return BT::NodeStatus::FAILURE;
    }

    // 从黑板获取站点映射表
    auto stations = config().blackboard->get<StationMap>("stations");
    auto it = stations.find(station_id);
    if (it == stations.end())
    {
        RCLCPP_ERROR(node_->get_logger(), "NavigateToStation: 站点不存在 [%s]",
                     station_id.c_str());
        return BT::NodeStatus::FAILURE;
    }

    const auto & pose = it->second.pose;

    // 构造 Nav2 导航目标
    NavigateToPose::Goal goal_msg;
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = node_->now();
    goal_msg.pose.pose.position.x = pose.x;
    goal_msg.pose.pose.position.y = pose.y;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, pose.yaw);
    goal_msg.pose.pose.orientation = tf2::toMsg(q);

    RCLCPP_INFO(node_->get_logger(), "NavigateToStation: 导航至 [%s] (%.2f, %.2f)",
                station_id.c_str(), pose.x, pose.y);

    // 发送导航目标
    auto send_goal_options =
        rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    send_goal_options.feedback_callback =
        [this, station_id](GoalHandle::SharedPtr,
                           const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                             "导航至 [%s] 剩余距离: %.2f 米",
                             station_id.c_str(), feedback->distance_remaining);
    };

    // 使用 result_callback 接收结果，避免在 onRunning 中阻塞等待
    result_ready_ = false;
    send_goal_options.result_callback =
        [this](const GoalHandle::WrappedResult &)
    {
        result_ready_ = true;
    };

    auto goal_future = nav_client_->async_send_goal(goal_msg, send_goal_options);

    // 等待目标被接受
    if (goal_future.wait_for(5s) != std::future_status::ready)
    {
        RCLCPP_ERROR(node_->get_logger(), "NavigateToStation: 目标发送超时");
        return BT::NodeStatus::FAILURE;
    }

    goal_handle_ = goal_future.get();
    if (!goal_handle_)
    {
        RCLCPP_ERROR(node_->get_logger(), "NavigateToStation: 目标被拒绝");
        return BT::NodeStatus::FAILURE;
    }

    // 保存 result future 供 onRunning 检查
    result_future_ = nav_client_->async_get_result(goal_handle_);

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateToStation::onRunning()
{
    // 检查导航是否完成
    if (!result_ready_)
    {
        return BT::NodeStatus::RUNNING;
    }

    // result_callback 已触发，获取结果
    if (result_future_.wait_for(0s) != std::future_status::ready)
    {
        return BT::NodeStatus::RUNNING;
    }

    const auto result = result_future_.get();
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
        RCLCPP_INFO(node_->get_logger(), "NavigateToStation: 导航成功");
        return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_WARN(node_->get_logger(), "NavigateToStation: 导航失败 (code=%d)",
                static_cast<int>(result.code));
    return BT::NodeStatus::FAILURE;
}

void NavigateToStation::onHalted()
{
    // 取消正在执行的导航
    if (goal_handle_)
    {
        RCLCPP_INFO(node_->get_logger(), "NavigateToStation: 取消导航");
        nav_client_->async_cancel_goal(goal_handle_);
        goal_handle_.reset();
    }
}

}  // namespace delivery_core

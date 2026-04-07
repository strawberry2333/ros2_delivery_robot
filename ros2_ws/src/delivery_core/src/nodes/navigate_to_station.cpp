/**
 * @file navigate_to_station.cpp
 * @brief NavigateToStation BT 叶节点实现。
 *
 * 这是单次配送流程里最核心的外部动作之一：把“去取货点/去送货点”的
 * 意图转换为 Nav2 的导航目标，并在结果回来前保持 RUNNING。
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
  // 构造函数不发起任何导航，只保存节点和 action client，真正动作在 onStart().
}

BT::NodeStatus NavigateToStation::onStart()
{
  // 先从端口取站点 ID；没有目标站点，就无法继续导航。
  std::string station_id;
  if (!getInput("station_id", station_id)) {
    RCLCPP_ERROR(node_->get_logger(), "NavigateToStation: 缺少 station_id 端口");
    return BT::NodeStatus::FAILURE;
  }

  // 站点坐标来自黑板，黑板通常由 delivery_executor 在 on_configure 阶段注入。
  // 使用 const 引用避免每次 onStart 时拷贝整个站点表。
  const auto & stations = config().blackboard->get<StationMap>("stations");
  auto it = stations.find(station_id);
  if (it == stations.end()) {
    RCLCPP_ERROR(node_->get_logger(), "NavigateToStation: 站点不存在 [%s]",
                     station_id.c_str());
    return BT::NodeStatus::FAILURE;
  }

  const auto & pose = it->second.pose;

  // 将仓库站点位姿转换成 Nav2 可消费的目标。
  std::string map_frame = "map";
  getInput("map_frame", map_frame);
  NavigateToPose::Goal goal_msg;
  goal_msg.pose.header.frame_id = map_frame;
  goal_msg.pose.header.stamp = node_->now();
  goal_msg.pose.pose.position.x = pose.x;
  goal_msg.pose.pose.position.y = pose.y;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, pose.yaw);
  goal_msg.pose.pose.orientation = tf2::toMsg(q);

  // 这条日志是调试导航链路最关键的输出之一，能直接看到目标站点和坐标。
  RCLCPP_INFO(node_->get_logger(), "NavigateToStation: 导航至 [%s] (%.2f, %.2f)",
                station_id.c_str(), pose.x, pose.y);

  // 配置 action 回调：
  // - feedback_callback 用于打印剩余距离
  // - result_callback 用于在结果到达后让 onRunning() 结束 RUNNING 状态
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

  // 使用 result_callback 接收结果，避免在 onRunning() 里阻塞等待 action future。
  result_ready_ = false;
  const uint64_t current_gen = goal_generation_.fetch_add(1, std::memory_order_acq_rel) + 1;
  send_goal_options.result_callback =
    [this, current_gen](const GoalHandle::WrappedResult &)
    {
      // 只有当 generation 匹配时才标记完成，避免 RetryUntilSuccessful 下
      // 旧 goal 的 late callback 误将新导航标记为完成。
      if (goal_generation_.load(std::memory_order_acquire) == current_gen) {
        result_ready_ = true;
      }
    };

  auto goal_future = nav_client_->async_send_goal(goal_msg, send_goal_options);

  // 这里等待目标是否被 Nav2 接受。若 goal 本身都没被接受，后面就没有继续轮询的意义。
  if (goal_future.wait_for(5s) != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(), "NavigateToStation: 目标发送超时");
    return BT::NodeStatus::FAILURE;
  }

  goal_handle_ = goal_future.get();
  if (!goal_handle_) {
    RCLCPP_ERROR(node_->get_logger(), "NavigateToStation: 目标被拒绝");
    return BT::NodeStatus::FAILURE;
  }

  // 将结果 future 保存下来，后续在 onRunning() 中轮询完成状态。
  result_future_ = nav_client_->async_get_result(goal_handle_);

  // 只要 goal 已成功发出，导航就是一个异步执行过程，因此返回 RUNNING。
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateToStation::onRunning()
{
  // 先看结果回调是否已经到达；未到达则保持 RUNNING。
  if (!result_ready_) {
    return BT::NodeStatus::RUNNING;
  }

  // result_callback 已触发，但 future 可能还没进入可取状态，继续保持 RUNNING。
  if (result_future_.wait_for(0s) != std::future_status::ready) {
    return BT::NodeStatus::RUNNING;
  }

  const auto result = result_future_.get();
  // Nav2 成功到站则返回 SUCCESS，交给父树进入下一阶段。
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(node_->get_logger(), "NavigateToStation: 导航成功");
    return BT::NodeStatus::SUCCESS;
  }

  // 其余结果都视为导航失败，由父树决定是重试还是终止任务。
  RCLCPP_WARN(node_->get_logger(), "NavigateToStation: 导航失败 (code=%d)",
                static_cast<int>(result.code));
  return BT::NodeStatus::FAILURE;
}

void NavigateToStation::onHalted()
{
  // 行为树不再需要这个节点时，显式取消 goal，避免旧导航继续跑。
  if (goal_handle_) {
    RCLCPP_INFO(node_->get_logger(), "NavigateToStation: 取消导航");
    nav_client_->async_cancel_goal(goal_handle_);
    goal_handle_.reset();
  }
}

}  // namespace delivery_core

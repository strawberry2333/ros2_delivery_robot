#pragma once

/**
 * @file navigate_to_station.hpp
 * @brief BT 叶节点：导航到指定站点。
 *
 * 这是单次配送链路里的主导航节点，通常会出现在“去取货点”和“去送货点”
 * 两个阶段。它从黑板读取站点 ID，查找站点坐标，再通过 Nav2 的
 * `NavigateToPose` action 发起导航。
 *
 * 业务语义：
 * - `RUNNING`：导航请求已发出，正在等待 Nav2 执行结果
 * - `SUCCESS`：机器人已到达目标站点
 * - `FAILURE`：站点不存在、目标被拒绝、发送超时或导航失败
 *
 * 参考 Ros2Learning/move_base_node.cpp 的 StatefulActionNode 模式。
 */

#include <atomic>
#include <string>
#include <unordered_map>

#include "behaviortree_cpp/action_node.h"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace delivery_core
{

/// 2D 位姿，用于仓库地图里的站点坐标存储。
struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

/// 站点描述，包含站点 ID、位姿和站点类型。
struct Station
{
  std::string id;
  Pose2D pose;
  uint8_t type{0};
};

/// 站点映射表类型别名，便于从黑板按 ID 快速查找站点。
using StationMap = std::unordered_map<std::string, Station>;

/**
 * @brief 导航到指定站点的 BT 叶节点（StatefulActionNode）
 *
 * 输入端口：
 * - `station_id`：目标站点 ID
 *
 * 黑板依赖：
 * - `stations`：站点映射表，键是站点 ID，值是站点坐标和类型
 *
 * 这个节点是 StatefulActionNode，因为导航是异步动作：
 * 先发 goal，再在 `onRunning()` 里轮询结果，期间行为树可继续由框架调度。
 */
class NavigateToStation : public BT::StatefulActionNode
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  NavigateToStation(
    const std::string & name,
    const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node,
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("station_id", "目标站点 ID"),
      BT::InputPort<std::string>("map_frame", "map", "导航目标坐标系"),
    };
  }

  /**
   * @brief 发送导航目标。
   *
   * 在这里完成站点查找、Pose 组装、goal 发送和目标接受检查。
   * 如果任何一步失败，节点直接返回 FAILURE；如果目标已成功发出，
   * 则返回 RUNNING，把后续结果交给 onRunning() 处理。
   */
  BT::NodeStatus onStart() override;

  /**
   * @brief 轮询导航结果。
   *
   * 这个阶段不再发 goal，只检查 action result 是否已经到达。
   * 完成后根据 Nav2 的结果码决定返回 SUCCESS 还是 FAILURE。
   */
  BT::NodeStatus onRunning() override;

  /**
   * @brief 行为树外部终止该节点时的清理入口。
   *
   * 若导航仍在执行，则发送 cancel 请求，避免旧 goal 继续占用 Nav2。
   */
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;

  GoalHandle::SharedPtr goal_handle_;
  std::shared_future<GoalHandle::WrappedResult> result_future_;
  std::atomic<bool> result_ready_{false};
};

}  // namespace delivery_core

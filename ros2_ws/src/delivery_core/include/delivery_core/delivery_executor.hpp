#pragma once

/**
 * @file delivery_executor.hpp
 * @brief 配送执行器生命周期节点声明（BT 宿主）。
 *
 * 作为 ExecuteDelivery Action Server，接收 delivery_manager 发来的配送请求，
 * 使用 BehaviorTree.CPP 驱动行为树执行完整的配送流程。
 *
 * 已实现为 LifecycleNode，由 lifecycle_manager 管理启动顺序。
 * 生命周期：
 *   Unconfigured → on_configure (加载站点、注册 BT) → Inactive
 *   Inactive → on_activate (创建 Action Server) → Active
 *   Active → on_deactivate (停止 BT、销毁 Action Server) → Inactive
 *   Inactive → on_cleanup (清除数据) → Unconfigured
 */

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "delivery_interfaces/action/execute_delivery.hpp"
#include "delivery_interfaces/msg/delivery_order.hpp"
#include "delivery_interfaces/msg/delivery_status.hpp"

#include "delivery_core/nodes/navigate_to_station.hpp"

namespace delivery_core
{

class DeliveryExecutor : public rclcpp_lifecycle::LifecycleNode
{
public:
  using ExecuteDelivery = delivery_interfaces::action::ExecuteDelivery;
  using GoalHandleExecuteDelivery = rclcpp_action::ServerGoalHandle<ExecuteDelivery>;
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using DeliveryStatus = delivery_interfaces::msg::DeliveryStatus;
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  DeliveryExecutor();
  ~DeliveryExecutor() override;

    // ====== 生命周期回调 ======
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  friend class DeliveryExecutorTestAccess;

    // ====== 配置加载 ======
  bool load_station_config(const std::string & path);

    // ====== BT 工厂初始化 ======
  void register_bt_nodes();

    // ====== 线程/停机管理 ======
  void request_bt_stop();
  void join_bt_thread();
  void stop_bt_helper_node();
  bool wait_for_nav2_active(std::chrono::seconds timeout);
  void clear_active_goal(
    const std::shared_ptr<GoalHandleExecuteDelivery> & goal_handle);

    // ====== Action Server 回调 ======
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ExecuteDelivery::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleExecuteDelivery> goal_handle);

  void handle_accepted(
    const std::shared_ptr<GoalHandleExecuteDelivery> goal_handle);

    // ====== BT 执行 ======
  void execute_bt(const std::shared_ptr<GoalHandleExecuteDelivery> goal_handle);

    // ====== ROS 参数 ======
  std::string station_config_path_;
  std::string tree_file_path_;
  double battery_drain_per_delivery_{15.0};

    // ====== 电池模拟 ======
    /// 当前电量百分比（BT 执行线程和 cleanup 均会访问，使用原子操作保护）
  std::atomic<double> battery_level_{100.0};

    // ====== 站点数据 ======
  StationMap stations_;

    // ====== BT 工厂 ======
  BT::BehaviorTreeFactory factory_;

    // ====== 确认标志 ======
  std::atomic<bool> load_confirmed_{false};
  std::atomic<bool> unload_confirmed_{false};

    // ====== 执行状态 ======
    /// 标记当前是否已有已接受但未结束的 goal，避免 handle_goal/handle_accepted 之间的竞态窗口
  std::atomic<bool> goal_inflight_{false};
  std::atomic<bool> executing_{false};
  std::atomic<bool> stop_requested_{false};
  std::mutex execution_mutex_;
  std::shared_ptr<GoalHandleExecuteDelivery> active_goal_handle_;

    /// BT 执行线程（替代 detach，确保节点关闭前线程已完成）
  std::thread bt_execution_thread_;

    // ====== 辅助节点 ======
    /// BT 节点需要 rclcpp::Node::SharedPtr（LifecycleNode 不继承自 Node），
    /// 因此创建一个轻量辅助节点供 BT 节点使用
  rclcpp::Node::SharedPtr bt_node_;

    /// 辅助节点的 executor 和 spin 线程，确保 Nav2 action client 回调能被处理
  rclcpp::executors::SingleThreadedExecutor::SharedPtr bt_node_executor_;
  std::thread bt_node_spin_thread_;

    // ====== ROS 通信接口 ======
  rclcpp_action::Server<ExecuteDelivery>::SharedPtr action_server_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp::Publisher<DeliveryStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    // ====== 确认服务端 ======
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr confirm_load_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr confirm_unload_srv_;

    // ====== 回调组 ======
  rclcpp::CallbackGroup::SharedPtr service_cb_group_;
};

}  // namespace delivery_core

#pragma once

/**
 * @file delivery_executor.hpp
 * @brief 配送执行器生命周期节点声明（BT 宿主）。
 *
 * 这个节点是“单订单执行层”，不负责订单队列和调度，只负责把一张已经
 * 通过校验的配送订单，真正执行成一次完整的任务流程。
 *
 * 它的输入来自 delivery_manager 的 ExecuteDelivery action 请求；
 * 它的执行引擎是 BehaviorTree.CPP；
 * 它的外部依赖是 Nav2 的 navigate_to_pose action，以及装货/卸货确认服务。
 *
 * 已实现为 LifecycleNode，由 lifecycle_manager 管理启动顺序。
 * 这样做的目的不是“形式化”，而是确保：
 * - 先加载站点配置和 BT 节点，再对外开放 action server
 * - 先确认 Nav2 可用，再允许 manager 发送配送请求
 * - 关闭时能按生命周期收敛 BT 线程、action server 和辅助节点
 *
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
  /**
   * @brief 单订单配送 action 类型。
   *
   * manager 通过这个 action 把“我要执行这单配送”的请求交给 executor，
   * executor 则通过反馈和最终结果把执行进度、完成情况、失败原因返回回去。
   */
  using ExecuteDelivery = delivery_interfaces::action::ExecuteDelivery;
  /// Action Server 侧的 goal handle，用于接受、取消、完成当前配送任务。
  using GoalHandleExecuteDelivery = rclcpp_action::ServerGoalHandle<ExecuteDelivery>;
  /// Nav2 的目标位姿 action 类型，BT 节点会用它驱动机器人到站点。
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  /// 对外发布的配送状态消息类型，供 manager 和外部监控订阅。
  using DeliveryStatus = delivery_interfaces::msg::DeliveryStatus;
  /// 生命周期回调的统一返回类型。
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /**
   * @brief 构造函数。
   *
   * 构造阶段只做“参数声明”和最轻量的对象初始化，不做重资源加载。
   * 真正的站点加载、BT 注册、通信接口创建都放在 on_configure 里完成，
   * 这样可以让生命周期语义保持清晰，也便于生命周期管理器控制启动顺序。
   */
  DeliveryExecutor();
  /**
   * @brief 析构函数。
   *
   * 重点不是释放一个普通类对象，而是确保 BT 执行线程、辅助节点 spin 线程、
   * 以及仍然持有的 goal handle 都能在节点销毁前有序收敛。
   */
  ~DeliveryExecutor() override;

  // ====== 生命周期回调 ======
  /**
   * @brief LifecycleNode 的配置阶段。
   *
   * 这里负责把 executor 从“仅仅存在”变成“可以执行配送任务”的状态：
   * - 创建 BT 叶节点依赖的辅助 Node 和 executor
   * - 创建 Nav2 action client
   * - 创建状态发布器和装货/卸货确认服务
   * - 读取站点配置
   * - 注册 BT 节点到 factory
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief LifecycleNode 的激活阶段。
   *
   * 只有在 Nav2 action server 和 bt_navigator 都可用后，才会真正创建
   * ExecuteDelivery action server。这样可以避免 manager 看到“能接单”
   * 但执行链路其实还没准备好的假就绪状态。
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief LifecycleNode 的停用阶段。
   *
   * 这里会停止当前 BT 执行、回收 action server，并把执行状态清理干净，
   * 以便后续重新 activate 时从一个明确状态重新开始。
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief LifecycleNode 的清理阶段。
   *
   * 与 deactivate 相比，cleanup 更进一步：它要释放已加载的站点数据、
   * 重置电量和服务/发布器引用，并注销 BT 节点 builder。
   */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief LifecycleNode 的关闭阶段。
   *
   * shutdown 是节点生命周期的最终收口动作，通常发生在进程退出或系统关闭时。
   * 这里会优先保证后台线程和辅助节点退出干净，避免残留 spin 线程影响进程终止。
   */
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  friend class DeliveryExecutorTestAccess;

  // ====== 配置加载 ======
  /**
   * @brief 从 YAML 加载站点配置。
   *
   * 站点配置是 executor 识别 pickup / dropoff / charge 站点的基础数据。
   * 没有这份数据，BT 即使启动也无法判断目标站点是否合法。
   */
  bool load_station_config(const std::string & path);

  // ====== BT 工厂初始化 ======
  /**
   * @brief 把 BT 叶节点注册进 factory。
   *
   * 这些节点拼起来以后，才构成真正的配送流程：
   * - NavigateToStation：导航到目标站点
   * - DockAtStation：在站点前做微调停靠
   * - WaitForConfirmation：等待人工确认装/卸货
   * - ReportDeliveryStatus：把执行状态写到黑板和状态话题
   * - CheckBattery：在任务前做电量门槛检查
   */
  void register_bt_nodes();

  // ====== 线程/停机管理 ======
  /**
   * @brief 请求 BT 执行线程尽快停止。
   *
   * 这是一个“软停止”信号，实际的停机动作会在 BT tick 循环中检查到
   * stop_requested_ 后再执行。
   */
  void request_bt_stop();
  /**
   * @brief 等待 BT 执行线程退出。
   *
   * 不能直接在析构或 deactivate 中粗暴 detach，否则 goal 结果、反馈发布、
   * 以及 active_goal_handle_ 清理都可能和进程退出竞争。
   */
  void join_bt_thread();
  /**
   * @brief 停止 BT 辅助节点及其 executor。
   *
   * 该辅助节点专门承载 BT 叶节点内部依赖的普通 rclcpp::Node 能力，
   * 例如 Nav2 action client 的回调处理。
   */
  void stop_bt_helper_node();
  /**
   * @brief 等待 Nav2 的 bt_navigator 进入 Active。
   *
   * executor 对外开放 action server 之前，必须先确认底层导航栈已经 ready。
   * 这样 manager 才不会把订单交给一个实际上还不能导航的系统。
   */
  bool wait_for_nav2_active(std::chrono::seconds timeout);
  /**
   * @brief 清理当前活跃 goal 的缓存指针。
   *
   * 这个函数的主要作用是避免 goal_handle 生命周期与 BT 线程收尾过程冲突。
   */
  void clear_active_goal(
    const std::shared_ptr<GoalHandleExecuteDelivery> & goal_handle);

  // ====== Action Server 回调 ======
  /**
   * @brief 接收配送请求时的门禁判断。
   *
   * 这里做的是“能不能接这单”的判断，而不是“怎么执行这单”的判断。
   * 典型检查包括站点合法性和单任务并发约束。
   */
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ExecuteDelivery::Goal> goal);

  /**
   * @brief 接收取消请求。
   *
   * 只要 action 层收到取消请求，就允许进入取消流程；真正是否能立即终止，
   * 由 BT tick 循环和各节点的停止逻辑完成。
   */
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleExecuteDelivery> & goal_handle);

  /**
   * @brief 接管已接受的 goal。
   *
   * 这里不会在 executor 线程里直接跑 BT，而是拉起独立线程执行，
   * 避免阻塞 action server 回调和生命周期回调。
   */
  void handle_accepted(
    const std::shared_ptr<GoalHandleExecuteDelivery> goal_handle);

  // ====== BT 执行 ======
  /**
   * @brief 执行一单配送的完整 BT 循环。
   *
   * 该函数是 executor 的核心：它会把订单信息写入黑板，创建 BT 实例，
   * 以固定频率 tick 行为树，并把黑板上的状态回传为 action feedback。
   */
  void execute_bt(const std::shared_ptr<GoalHandleExecuteDelivery> goal_handle);

  // ====== ROS 参数 ======
  /**
   * @brief 站点配置文件路径。
   *
   * 由 launch 文件注入，决定 executor 能识别哪些 pickup/dropoff/charge 站点。
   */
  std::string station_config_path_;
  /// BT XML 文件路径，决定单次配送流程如何被编排。
  std::string tree_file_path_;
  /// Nav2 导航 action server 名称，对应 launch/yaml 中的 nav2_action_name。
  std::string nav2_action_name_{"navigate_to_pose"};
  /// 每单完成后扣减的模拟电量，demo 级别参数。
  double battery_drain_per_delivery_{15.0};
  /// 电量最低阈值（百分比），低于此值拒绝配送。与 BT XML 中 CheckBattery 保持同步。
  double battery_threshold_{20.0};

  // ====== 电池模拟 ======
  /// 当前电量百分比，BT 执行线程和 cleanup 都可能读写，因此使用原子变量。
  std::atomic<double> battery_level_{100.0};

  // ====== 站点数据 ======
  /// 从 YAML 加载后的站点表，供 BT 节点和 goal 校验使用。
  StationMap stations_;

  // ====== BT 工厂 ======
  /// 负责按 XML 和节点注册信息构建实际行为树。
  BT::BehaviorTreeFactory factory_;

  // ====== 确认标志 ======
  /// 由 /confirm_load 服务置位，供 WaitForConfirmation 节点读取。
  std::atomic<bool> load_confirmed_{false};
  /// 由 /confirm_unload 服务置位，供 WaitForConfirmation 节点读取。
  std::atomic<bool> unload_confirmed_{false};
  /// 当前执行阶段：0=无, 1=等待装货确认, 2=等待卸货确认。
  /// 服务回调据此拒绝不在对应阶段的过早/错误确认。
  std::atomic<uint8_t> current_phase_{0};

  // ====== 执行状态 ======
  /// 标记当前是否已有已接受但未结束的 goal，避免 handle_goal / handle_accepted 的竞态窗口。
  std::atomic<bool> goal_inflight_{false};
  /// 标记 BT 正在执行中，供测试和停机流程判断。
  std::atomic<bool> executing_{false};
  /// 外部停止请求标志，例如 deactivate / shutdown 时触发。
  std::atomic<bool> stop_requested_{false};
  /// 保护 active_goal_handle_ 和 BT 执行线程收尾过程。
  std::mutex execution_mutex_;
  /// 当前活跃的 goal handle，用于取消和收尾时判断。
  std::shared_ptr<GoalHandleExecuteDelivery> active_goal_handle_;

  /// BT 执行线程。这里显式持有线程对象，而不是 detach，便于生命周期收敛。
  std::thread bt_execution_thread_;

  // ====== 辅助节点 ======
  /// BT 节点需要普通 Node 接口，而 LifecycleNode 本身不直接满足这一点。
  rclcpp::Node::SharedPtr bt_node_;

  /// 辅助节点对应的 executor，专门用来处理 BT 叶节点依赖的回调。
  rclcpp::executors::SingleThreadedExecutor::SharedPtr bt_node_executor_;
  /// 辅助节点的 spin 线程。
  std::thread bt_node_spin_thread_;

  // ====== ROS 通信接口 ======
  /// 对外提供的配送 action server。
  rclcpp_action::Server<ExecuteDelivery>::SharedPtr action_server_;
  /// 指向 Nav2 navigate_to_pose 的 action client。
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  /// 停靠节点使用的速度话题发布器。
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  // ====== 确认服务端 ======
  /// 装货确认服务，对应 WaitForConfirmation 的 load 分支。
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr confirm_load_srv_;
  /// 卸货确认服务，对应 WaitForConfirmation 的 unload 分支。
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr confirm_unload_srv_;

  // ====== 回调组 ======
  /// 将确认服务和 action 回调隔离到独立回调组，减少互相阻塞风险。
  rclcpp::CallbackGroup::SharedPtr service_cb_group_;
};

}  // namespace delivery_core

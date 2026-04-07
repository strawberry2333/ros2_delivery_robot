#pragma once

/**
 * @file delivery_manager.hpp
 * @brief 配送管理节点声明。
 *
 * 该节点对应的是“订单级调度器”，不是具体导航执行器。
 * 它负责接收配送订单、维护优先级队列、协调 delivery_executor 与 Nav2，
 * 并将单次配送流程的状态统一发布给外部监控系统。
 *
 * 状态机: kIdle → kGoingToPickup → kWaitingLoad → kGoingToDropoff → kWaitingUnload → kComplete
 * 失败或取消会回到可接单状态，由下一单继续驱动系统前进。
 *
 * 设计思路：
 * - 本节点是配送系统的中枢调度器，负责订单队列管理和配送调度。
 *   实际配送执行委托给 delivery_executor 节点（通过 ExecuteDelivery Action）。
 *   这样 manager 只关心订单编排，不直接耦合行为树细节或 Nav2 状态。
 * - ROS 回调（订单提交、取消、报告查询）在 MultiThreadedExecutor
 *   的后台线程中并发处理，通过互斥锁与主线程安全交互。
 *   订单队列、当前执行中的订单、当前 action goal 句柄分别受不同 mutex 保护，
 *   避免服务回调和主循环之间的数据竞争。
 */

#include <deque>
#include <cmath>
#include <future>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

// ROS 2 核心库
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// ROS 2 标准消息
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

// 项目自定义消息/服务/动作接口
#include "delivery_interfaces/msg/delivery_order.hpp"
#include "delivery_interfaces/msg/delivery_status.hpp"
// delivery_interfaces/msg/station_info.hpp — 当前未使用（RESERVED），不再引入
#include "delivery_interfaces/srv/submit_order.hpp"
#include "delivery_interfaces/srv/cancel_order.hpp"
#include "delivery_interfaces/srv/get_delivery_report.hpp"
#include "delivery_interfaces/action/execute_delivery.hpp"

// YAML 配置文件解析库
#include "yaml-cpp/yaml.h"

// 项目公共类型
#include "delivery_core/types.hpp"

namespace delivery_core
{

/**
 * @brief 配送管理节点
 *
 * 核心职责：
 * 1. 从 YAML 加载站点配置（站点 ID、坐标、类型）
 * 2. 通过 SubmitOrder 服务接收订单，维护按优先级排序的订单队列
 * 3. 按序执行配送：导航取货点 → 等待装货确认 → 导航送货点 → 等待卸货确认
 * 4. 发布实时 DeliveryStatus 状态到 /delivery_status 话题
 * 5. 支持订单取消（仅限队列中尚未执行的订单）和配送报告查询
 *
 * 继承自 rclcpp::Node，作为 ROS 2 标准节点运行。
 * 需要配合 MultiThreadedExecutor 使用，以确保回调与主循环不互相阻塞。
 */
class DeliveryManager : public rclcpp::Node
{
public:
    // --- 类型别名，简化模板类型的使用 ---
  using PoseStamped = geometry_msgs::msg::PoseStamped;                     ///< 带时间戳的位姿消息
  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;    ///< 带协方差的位姿（用于初始化 AMCL）
  using DeliveryOrder = delivery_interfaces::msg::DeliveryOrder;           ///< 配送订单消息
  using DeliveryStatus = delivery_interfaces::msg::DeliveryStatus;         ///< 配送状态消息
  // StationInfo.msg 当前未使用（RESERVED），类型别名已移除
  using ExecuteDelivery = delivery_interfaces::action::ExecuteDelivery;    ///< 配送执行 Action 类型
  using ExecuteDeliveryGoalHandle = rclcpp_action::ClientGoalHandle<ExecuteDelivery>;   ///< 配送 Action Goal 句柄
  using GoalResultFuture = std::shared_future<ExecuteDeliveryGoalHandle::WrappedResult>;  ///< 配送 Action 结果 future 类型别名
  using SubmitOrderSrv = delivery_interfaces::srv::SubmitOrder;         ///< 提交订单服务类型
  using CancelOrderSrv = delivery_interfaces::srv::CancelOrder;         ///< 取消订单服务类型
  using GetDeliveryReportSrv = delivery_interfaces::srv::GetDeliveryReport;  ///< 获取配送报告服务类型

    /**
     * @brief 构造函数，初始化所有参数、通信接口和回调组。
     *
     * 执行以下初始化：
     * - 声明并读取 ROS 参数（坐标系、超时时间、初始位姿等）
     * - 创建 Reentrant 回调组，使服务回调可以并发执行
     * - 创建状态发布器、初始位姿发布器和订单管理相关服务端
     * - 创建 ExecuteDelivery Action Client，用于把单单执行委托给 executor
     * - 尝试预加载站点配置，便于启动时就发现配置错误
     *
     * @note 构造函数不会启动配送循环，需要调用 run() 方法。
     * @note 这个节点本身不执行导航，也不直接等待装卸确认；这些动作都由
     *       delivery_executor 和其 BT 节点完成。
     */
  explicit DeliveryManager(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    /**
     * @brief 主运行逻辑，阻塞执行配送循环。
     *
     * 执行流程：
     * 1. 等待仿真时钟就绪（如果启用 use_sim_time）
     * 2. 加载站点配置文件
     * 3. 发布初始位姿，帮助 AMCL 进入正确的位姿估计区域
     * 4. 等待 delivery_executor 的 action server 可用
     * 5. 等待 map→base_link TF 链路就绪
     * 6. 进入主循环：以 2Hz 频率轮询订单队列，取出订单并执行配送
     *
     * @note 本方法会阻塞调用线程，直到 rclcpp::ok() 返回 false（节点关闭）。
     *       必须在另一个线程中运行 executor.spin() 来处理 ROS 回调。
     * @note 这里采用“准备阶段先完成，再进入订单循环”的顺序，是为了避免
     *       订单被过早接受后因为 Nav2 / TF / 初始位姿未就绪而失败。
     */
  void run();

    /// 测试辅助：跳过 run() 的启动检查，直接标记系统就绪。
    /// 单元测试中可用它绕过仿真时钟、TF 和 action server 等依赖。
  void set_system_ready() {system_ready_.store(true);}

private:
  friend class DeliveryManagerTestAccess;

    /**
     * @brief 配送任务状态枚举
     *
     * 描述单个配送订单在执行过程中的生命周期阶段。
     * 状态转换是单向线性的：kIdle → kGoingToPickup → kWaitingLoad →
     * kGoingToDropoff → kWaitingUnload → kComplete，任一阶段失败则跳转到 kFailed。
     *
     * 这里使用内部枚举而不是直接复用 DeliveryStatus.msg 的数值，
     * 是为了让业务代码表达“流程语义”，同时在 publish_status() 中统一映射到消息常量。
     */
  enum class DeliveryState
  {
    kIdle,                 ///< 空闲，订单已入队但尚未开始执行
    kGoingToPickup,        ///< 正在前往取货站点（Nav2 导航中）
    kWaitingLoad,          ///< 到达取货点，等待人工装货确认（通过 /confirm_load 服务触发）
    kGoingToDropoff,       ///< 正在前往送货站点（Nav2 导航中）
    kWaitingUnload,        ///< 到达送货点，等待人工卸货确认（通过 /confirm_unload 服务触发）
    kComplete,             ///< 当前订单配送完成（全部阶段成功）
    kFailed,               ///< 当前订单配送失败（导航失败、确认超时等）
    kCanceled              ///< 当前订单在执行中被用户取消
  };

    // Pose2D、Station、StationMap 定义见 delivery_core/types.hpp

    /**
     * @brief 订单记录，含运行时状态追踪信息
     *
     * 封装了原始订单消息和配送执行过程中的状态信息。
     * 订单完成、失败或取消后会被移入 completed_orders_ 用于历史查询。
     * 这样 GetDeliveryReport 可以同时返回“等待执行”“正在执行”“已经结束”的订单。
     */
  struct OrderRecord
  {
    DeliveryOrder order;              ///< 原始订单消息（order_id, pickup_station, dropoff_station, priority）
    DeliveryState state{DeliveryState::kIdle};      ///< 当前配送状态，初始为空闲
    std::string error_msg;            ///< 失败时的错误描述信息，成功时为空
    std::string current_station;      ///< 当前关联站点（由 executor feedback 同步）
    float progress{0.0f};             ///< 当前进度（由 executor feedback 同步）
    rclcpp::Time start_time;          ///< 订单开始执行的时间戳，用于性能统计
    rclcpp::Time end_time;            ///< 订单完成的时间戳，入库时记录
  };

  struct LoggedStatus
  {
    std::string order_id;
    DeliveryState state{DeliveryState::kIdle};
    std::string station;
    float progress{0.0f};
    std::string error_msg;
  };

    // ====== 配置加载 ======

    /**
     * @brief 从 YAML 文件加载站点配置。
     *
     * 解析 YAML 文件中的 "stations" 列表，将每个站点的 ID、坐标、类型
     * 存入 stations_ 哈希表中，供后续订单验证和导航使用。
     * 站点配置是 manager 与 executor 共享的业务基础数据，必须在启动时就可用，
     * 否则提交订单和导航目标解析都无法成立。
     *
     * @param[in] path YAML 文件的绝对路径。
     * @return 加载成功且至少存在一个站点时返回 true；路径为空、文件解析失败、
     *         站点类型非法、站点 ID 重复等情况返回 false。
     *
     * @note YAML 格式示例：
     *   stations:
     *     - station_id: "station_A"
     *       x: 1.0
     *       y: 2.0
     *       yaw: 0.0
     *       station_type: 0
     */
  bool load_station_config(const std::string & path);

    // ====== 环境准备 ======

    /**
     * @brief 等待仿真时钟可用。
     *
     * 在 use_sim_time 模式下，ROS 节点的时钟依赖 /clock 话题。
     * Gazebo 启动后才会发布 /clock，因此需要等待时钟值变为非零。
     * 使用 steady_clock（挂钟时间）而非 ROS 时钟来计算超时，
     * 因为在等待期间 ROS 时钟尚不可用。
     * 这个等待是 manager 启动链路的一部分，不能省略，否则后续 now() 相关
     * 逻辑可能一直停留在 0，导致时间戳和超时判断失真。
     *
     * @return 时钟可用返回 true；超时（clock_wait_timeout_sec_）返回 false。
     */
  bool wait_for_time();

    /**
     * @brief 等待 TF 链路就绪 (map → base_link)。
     *
     * AMCL 定位节点启动后会发布 map→odom 的 TF 变换，
     * 结合 odom→base_link 的里程计 TF，形成完整的 map→base_link 链路。
     * 必须等此链路就绪后才能开始导航，否则 Nav2 无法确定机器人当前位置。
     * 这里并不负责持续监听 TF，只是在启动阶段做“可导航性”检查。
     *
     * @return 在超时时间（tf_wait_timeout_sec_）内链路就绪返回 true，否则返回 false。
     */
  bool wait_for_tf();

    /**
     * @brief 等待 /initialpose 至少有一个订阅者。
     *
     * manager 负责发布初始位姿给 AMCL；如果在 AMCL 尚未完成订阅前就一次性发完，
     * 启动链路会进入“定位未建立 -> Nav2 不激活 -> executor 不可用”的死等状态。
     * 这里显式等待订阅者出现，避免把初始定位建立建立在启动时序运气上。
     *
     * @param[in] timeout_sec 最长等待时间（秒）。
     * @return 在超时前检测到订阅者返回 true，否则返回 false。
     */
  bool wait_for_initial_pose_subscriber(double timeout_sec);


    /**
     * @brief 发布初始位姿到 /initialpose 话题。
     *
     * 向 AMCL 发布机器人的初始位姿估计，使定位粒子收敛到正确位置。
     * 连续发布 5 次（间隔 200ms），因为 AMCL 可能在启动瞬间丢失消息。
     * 协方差矩阵对角线设为 0.25，表示对初始位置有 0.5 米的不确定性。
     * 发布初始位姿的职责在 manager 这一侧，是为了尽早把“地图坐标系中的机器人位置”
     * 建立起来，让 Nav2 和 executor 后续都能拿到可用的 TF。
     *
     * @note 仅在参数 publish_initial_pose 为 true 时调用。
     *       如果使用真实机器人并依赖其他定位方式，应关闭此功能。
     */
  void publish_initial_pose();

    // ====== 状态管理 ======

    /**
     * @brief 更新并发布配送状态到 /delivery_status 话题。
     *
     * 构造 DeliveryStatus 消息并发布，同时打印日志。
     * 外部监控系统可以订阅此话题来追踪所有订单的实时状态。
     * 这是 manager 对外的主状态出口，executor 的反馈会先同步到内部状态，
     * 再经由这里统一对外发布，避免上层系统同时监听多个来源。
     *
     * @param[in] order_id 订单 ID。
     * @param[in] state 新的配送状态。
     * @param[in] station 当前关联的站点 ID（可选，默认空字符串）。
     * @param[in] progress 进度值，范围 [0.0, 1.0]（可选，默认 0.0）。
     * @param[in] error 错误信息（可选，默认空字符串，仅在失败时填充）。
     */
  void publish_status(
    const std::string & order_id, DeliveryState state,
    const std::string & station = "",
    float progress = 0.0f,
    const std::string & error = "");

    /**
     * @brief 将内部状态枚举转换为 DeliveryStatus 消息中的 uint8 常量。
     *
     * DeliveryStatus.msg 中定义了 STATE_IDLE、STATE_GOING_TO_PICKUP 等常量，
     * 本方法负责将 C++ 枚举映射到这些 ROS 消息常量。
     * 这样做的好处是：内部实现可以保持更清晰的业务状态名，
     * 而对外消息格式保持稳定，不会因为内部重构而影响订阅方。
     *
     * @param[in] state 内部状态枚举值。
     * @return 对应的 DeliveryStatus 消息常量值。
     */
  uint8_t state_to_msg(DeliveryState state) const;

    /**
     * @brief 将状态枚举转换为可读的中英文字符串，用于日志输出。
     *
     * @param[in] state 内部状态枚举值。
     * @return 格式为 "中文(English)" 的状态描述字符串。
     * 该函数只影响日志可读性，不影响状态机本身。
     */
  std::string state_to_string(DeliveryState state) const;

    // ====== 配送执行 ======

    /**
     * @brief 执行单个配送订单的完整流程。
     *
     * 按照 "导航取货点→等待装货→导航送货点→等待卸货" 四个阶段顺序执行。
     * 每个阶段开始前都会更新并发布状态，任一阶段失败则立即返回 false。
     * 真正的导航动作和等待确认逻辑不在 manager 内实现，而是委托给 executor
     * 和其内部行为树完成；manager 只负责发起、跟踪和收尾。
     *
     * @param[in,out] record 订单记录，执行过程中会更新其 state 和 error_msg 字段。
     * @return 全部四个阶段成功返回 true；任一阶段失败返回 false。
     *
     * @note 重试逻辑在 BT 层实现（RetryUntilSuccessful 包裹导航节点）。
     */
  bool execute_delivery(OrderRecord & record);

    /**
     * @brief 超时后等待 executor 进入终态，避免 manager 过早开始下一个订单。
     *
     * 当 action 取消已经发出时，executor 可能还需要一点时间完成内部清理。
     * 这里补等一小段时间，是为了让当前订单真正收尾后再进入下一单。
     *
     * @param[in] order_id 当前订单 ID，用于日志。
     * @param[in] result_future ExecuteDelivery 的结果 future。
     * @return 在等待窗口内进入终态返回 true，否则返回 false。
     */
  bool wait_for_terminal_result_after_cancel(
    const std::string & order_id,
    const GoalResultFuture & result_future);

    /**
     * @brief 等待 /execute_delivery Action Server 可用。
     *
     * delivery_executor 只有在完成 lifecycle configure + activate，并且确认 Nav2
     * 已经可用后，才会注册 Action Server。
     * 因此 manager 在正式接单前必须等待它上线。
     *
     * @return Action Server 可用返回 true；超时返回 false。
     */
  bool wait_for_executor_server();

    // ====== 服务回调 ======

    /**
     * @brief SubmitOrder 服务回调：接收并验证新的配送订单。
     *
     * 验证逻辑：
     * 1. order_id 不能为空
     * 2. pickup_station 和 dropoff_station 必须在已加载的站点配置中存在
     * 3. 取货站类型不能是 dropoff(1)，送货站类型不能是 pickup(0)
     * 4. order_id 在待执行队列和历史记录中均不能重复
     * 验证通过后，按优先级插入订单队列（高优先级在前）。
     * 这个服务是外部系统进入配送队列的唯一入口之一，因此必须做完整校验，
     * 不能把非法订单交给后面的 executor 或 BT 再处理。
     *
     * @param[in] request 包含 DeliveryOrder 消息的请求。
     * @param[out] response accepted=true 表示接受，否则 reason 字段含拒绝原因。
     */
  void handle_submit_order(
    const std::shared_ptr<SubmitOrderSrv::Request> request,
    std::shared_ptr<SubmitOrderSrv::Response> response);

    /**
     * @brief CancelOrder 服务回调：取消队列中尚未执行的订单。
     *
     * 在订单队列中查找匹配的 order_id，找到则移除。
     * 已经开始执行或已完成的订单无法取消。
     * 如果目标订单已经被当前执行器接手，则通过 action cancel 转交给 executor。
     *
     * @param[in] request 包含 order_id 的取消请求。
     * @param[out] response success=true 表示成功取消，否则 reason 字段含失败原因。
     */
  void handle_cancel_order(
    const std::shared_ptr<CancelOrderSrv::Request> request,
    std::shared_ptr<CancelOrderSrv::Response> response);

    /**
     * @brief GetDeliveryReport 服务回调：查询所有订单的配送状态报告。
     *
     * 返回排队中和已完成/失败的所有订单状态，用于外部系统监控。
     * 报告按内部存储分区拼接，而不是强制按提交时间排序；这一点需要和调用方约定。
     *
     * @param[in] request 空请求（无输入参数）。
     * @param[out] response reports 字段包含所有订单的 DeliveryStatus 列表。
     */
  void handle_get_report(
    const std::shared_ptr<GetDeliveryReportSrv::Request> request,
    std::shared_ptr<GetDeliveryReportSrv::Response> response);

    // ====== ROS 参数 ======
  std::string map_frame_;                       ///< 地图坐标系名称，默认 "map"，所有导航目标点都在此坐标系下
  std::string base_frame_;                      ///< 机器人本体坐标系名称，默认 "base_link"，用于 TF 查询
  std::string station_config_path_;             ///< 站点配置 YAML 文件路径，由 launch 文件传入
  double clock_wait_timeout_sec_{30.0};         ///< 等待仿真时钟 /clock 的超时秒数
  double tf_wait_timeout_sec_{15.0};            ///< 等待 TF 链路 (map→base_link) 就绪的超时秒数
  double navigation_timeout_sec_{120.0};        ///< 单次导航任务的超时秒数（超时后取消导航）
  double wait_confirmation_timeout_sec_{60.0};   ///< 等待装/卸货确认的超时秒数
  double action_server_wait_timeout_sec_{60.0};   ///< 等待 ExecuteDelivery Action Server 上线的超时秒数（executor 激活含 Nav2 等待，需留足余量）
  double cancel_completion_wait_timeout_sec_{10.0};   ///< manager 发送取消后等待 executor 进入终态的秒数
  double initial_x_{0.0};                       ///< 初始位姿 X 坐标（米），发布到 /initialpose
  double initial_y_{0.0};                       ///< 初始位姿 Y 坐标（米），发布到 /initialpose
  double initial_yaw_{0.0};                     ///< 初始位姿偏航角（弧度），发布到 /initialpose
  bool publish_initial_pose_{false};            ///< 是否在启动时发布初始位姿（仿真环境通常需要开启）
  bool use_sim_time_{true};                     ///< 是否使用仿真时钟（仿真环境设为 true，实机设为 false）

    // ====== 站点数据 ======
    /// 站点 ID → Station 映射表，启动时从 YAML 加载。
    /// 线程安全约定：stations_ 在 run() 阶段一次性加载完毕，system_ready_ 置 true 后
    /// 不再修改，因此服务回调中的只读访问无需额外加锁。
  std::unordered_map<std::string, Station> stations_;

    // ====== 订单队列 ======
    // 锁层级约定（嵌套加锁时必须按此顺序，禁止反向）：
    //   queue_mutex_ > current_order_mutex_ > goal_handle_mutex_
  std::deque<OrderRecord> order_queue_;                    ///< 待执行订单队列（按优先级排序，高优先级在前）
  std::vector<OrderRecord> completed_orders_;              ///< 已完成/失败订单的历史记录，用于报告查询
  std::mutex queue_mutex_;                                 ///< 订单队列互斥锁，保护 order_queue_ 和 completed_orders_ 的并发访问

    // ====== ROS 通信接口 ======
  rclcpp::Publisher<DeliveryStatus>::SharedPtr status_pub_;             ///< 配送状态发布器，发布到 /delivery_status
  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;   ///< 初始位姿发布器，发布到 /initialpose 供 AMCL 使用

    // ====== 服务端 ======
  rclcpp::Service<SubmitOrderSrv>::SharedPtr submit_order_srv_;           ///< 订单提交服务端 (/submit_order)
  rclcpp::Service<CancelOrderSrv>::SharedPtr cancel_order_srv_;           ///< 订单取消服务端 (/cancel_order)
  rclcpp::Service<GetDeliveryReportSrv>::SharedPtr get_report_srv_;       ///< 配送报告查询服务端 (/get_delivery_report)

    // ====== 配送执行 Action Client ======
  rclcpp_action::Client<ExecuteDelivery>::SharedPtr delivery_action_client_;   ///< ExecuteDelivery Action Client，调用 executor
  ExecuteDeliveryGoalHandle::SharedPtr current_goal_handle_;                   ///< 当前正在执行的配送 Goal 句柄（用于取消）
  std::mutex goal_handle_mutex_;                                               ///< 保护 current_goal_handle_ 的互斥锁

    // ====== 当前执行中的订单追踪 ======
  std::string current_order_id_;                                               ///< 当前正在执行的订单 ID（空字符串表示无执行中订单）
  std::optional<OrderRecord> current_order_;                                   ///< 当前正在执行的订单记录（用于报告查询）
  std::mutex current_order_mutex_;                                             ///< 保护 current_order_id_ 和 current_order_ 的互斥锁
  std::optional<LoggedStatus> last_logged_status_;                             ///< 上一次写入日志的状态快照，用于抑制重复日志
  std::mutex status_log_mutex_;                                                ///< 保护 last_logged_status_ 的互斥锁

    // ====== 系统就绪标志 ======
  std::atomic<bool> system_ready_{false};   ///< run() 完成启动检查后置 true，服务回调据此拒绝过早的请求

    // ====== 回调组 ======
    /**
     * @brief Reentrant 回调组，将所有服务回调分配到此组。
     *
     * 使用 Reentrant 类型而非默认的 MutuallyExclusive 类型，原因是：
     * - 主线程的 run() 方法中调用了 rclcpp::sleep_for() 等阻塞操作
     * - 如果服务回调与主线程在同一 MutuallyExclusive 组中，会导致死锁
     * - Reentrant 组允许多个回调并发执行，确保订单提交/取消等操作不被阻塞
     * - 这里的并发控制由更细粒度的 mutex 完成，而不是依赖回调组串行化
     */
  rclcpp::CallbackGroup::SharedPtr service_cb_group_;
};

}  // namespace delivery_core

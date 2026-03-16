#pragma once

/**
 * @file delivery_manager.hpp
 * @brief 配送管理节点声明。
 *
 * 负责接收配送订单、管理订单队列、协调导航与停靠确认流程。
 * 状态机: kIdle → kGoingToPickup → kWaitingLoad → kGoingToDropoff → kWaitingUnload → kComplete
 *
 * 设计思路：
 * - 本节点是配送系统的中枢调度器，负责订单队列管理和配送调度。
 *   实际配送执行委托给 delivery_executor 节点（通过 ExecuteDelivery Action）。
 * - ROS 回调（订单提交、取消、报告查询）在 MultiThreadedExecutor
 *   的后台线程中并发处理，通过互斥锁与主线程安全交互。
 */

#include <deque>
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
#include "nav2_msgs/action/navigate_to_pose.hpp"    // Nav2 导航 Action 接口

// 项目自定义消息/服务/动作接口
#include "delivery_interfaces/msg/delivery_order.hpp"
#include "delivery_interfaces/msg/delivery_status.hpp"
#include "delivery_interfaces/msg/station_info.hpp"
#include "delivery_interfaces/srv/submit_order.hpp"
#include "delivery_interfaces/srv/cancel_order.hpp"
#include "delivery_interfaces/srv/get_delivery_report.hpp"
#include "delivery_interfaces/action/execute_delivery.hpp"

// YAML 配置文件解析库
#include "yaml-cpp/yaml.h"

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
    using NavigateToPose = nav2_msgs::action::NavigateToPose;              ///< Nav2 导航 Action 类型
    using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;    ///< 导航目标句柄类型
    using PoseStamped = geometry_msgs::msg::PoseStamped;                   ///< 带时间戳的位姿消息
    using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;  ///< 带协方差的位姿（用于初始化 AMCL）
    using DeliveryOrder = delivery_interfaces::msg::DeliveryOrder;         ///< 配送订单消息
    using DeliveryStatus = delivery_interfaces::msg::DeliveryStatus;       ///< 配送状态消息
    using StationInfo = delivery_interfaces::msg::StationInfo;             ///< 站点信息消息
    using ExecuteDelivery = delivery_interfaces::action::ExecuteDelivery;  ///< 配送执行 Action 类型
    using ExecuteDeliveryGoalHandle = rclcpp_action::ClientGoalHandle<ExecuteDelivery>; ///< 配送 Action Goal 句柄

    /**
     * @brief 构造函数，初始化所有参数、通信接口和回调组。
     *
     * 执行以下初始化：
     * - 声明并读取 ROS 参数（坐标系、超时时间、初始位姿等）
     * - 创建 Reentrant 回调组，使服务回调可以并发执行
     * - 创建 Nav2 Action Client、状态发布器、初始位姿发布器
     * - 创建订单提交/取消/报告查询服务端
     * - 创建装货/卸货确认服务端（模拟人工操作）
     *
     * @note 构造函数不会启动配送循环，需要调用 run() 方法。
     */
    DeliveryManager();

    /**
     * @brief 主运行逻辑，阻塞执行配送循环。
     *
     * 执行流程：
     * 1. 等待仿真时钟就绪（如果启用 use_sim_time）
     * 2. 加载站点配置文件
     * 3. 等待 Nav2 Action Server 可用
     * 4. 发布初始位姿（如果启用）
     * 5. 等待 TF 链路就绪
     * 6. 进入主循环：以 2Hz 频率轮询订单队列，取出订单并执行配送
     *
     * @note 本方法会阻塞调用线程，直到 rclcpp::ok() 返回 false（节点关闭）。
     *       必须在另一个线程中运行 executor.spin() 来处理 ROS 回调。
     */
    void run();

private:
    /**
     * @brief 配送任务状态枚举
     *
     * 描述单个配送订单在执行过程中的生命周期阶段。
     * 状态转换是单向线性的：kIdle → kGoingToPickup → kWaitingLoad →
     * kGoingToDropoff → kWaitingUnload → kComplete，任一阶段失败则跳转到 kFailed。
     */
    enum class DeliveryState
    {
        kIdle,             ///< 空闲，订单已入队但尚未开始执行
        kGoingToPickup,    ///< 正在前往取货站点（Nav2 导航中）
        kWaitingLoad,      ///< 到达取货点，等待人工装货确认（通过 /confirm_load 服务触发）
        kGoingToDropoff,   ///< 正在前往送货站点（Nav2 导航中）
        kWaitingUnload,    ///< 到达送货点，等待人工卸货确认（通过 /confirm_unload 服务触发）
        kComplete,         ///< 当前订单配送完成（全部阶段成功）
        kFailed,           ///< 当前订单配送失败（导航失败、确认超时等）
        kCanceled          ///< 当前订单在执行中被用户取消
    };

    /**
     * @brief 2D 位姿，用于内部坐标存储
     *
     * 简化的位姿表示，仅包含平面坐标和朝向角。
     * 由于配送机器人在室内地面运动，不需要完整的 6DOF 位姿。
     */
    struct Pose2D
    {
        double x{0.0};     ///< X 坐标（米），相对于地图坐标系原点
        double y{0.0};     ///< Y 坐标（米），相对于地图坐标系原点
        double yaw{0.0};   ///< 偏航角（弧度），表示机器人到达站点后的朝向
    };

    /**
     * @brief 站点描述，从 YAML 配置文件加载
     *
     * 每个站点对应配送环境中的一个物理位置（取货点、送货点或充电桩）。
     * 站点数据在启动时一次性加载，运行期间不可动态修改。
     */
    struct Station
    {
        std::string id;       ///< 站点唯一标识符，如 "station_A"
        Pose2D pose;          ///< 站点在地图坐标系中的位姿
        uint8_t type{0};      ///< 站点类型：0=取货点(pickup), 1=送货点(dropoff), 2=充电桩(charge)
    };

    /**
     * @brief 订单记录，含运行时状态追踪信息
     *
     * 封装了原始订单消息和配送执行过程中的状态信息。
     * 订单完成或失败后会被移入 completed_orders_ 用于历史查询。
     */
    struct OrderRecord
    {
        DeliveryOrder order;          ///< 原始订单消息（order_id, pickup_station, dropoff_station, priority）
        DeliveryState state{DeliveryState::kIdle};  ///< 当前配送状态，初始为空闲
        std::string error_msg;        ///< 失败时的错误描述信息，成功时为空
        rclcpp::Time start_time;      ///< 订单开始执行的时间戳，用于性能统计
    };

    // ====== 配置加载 ======

    /**
     * @brief 从 YAML 文件加载站点配置。
     *
     * 解析 YAML 文件中的 "stations" 列表，将每个站点的 ID、坐标、类型
     * 存入 stations_ 哈希表中，供后续订单验证和导航使用。
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
     *
     * @return 在超时时间（tf_wait_timeout_sec_）内链路就绪返回 true，否则返回 false。
     */
    bool wait_for_tf();

    /**
     * @brief 等待 Nav2 action server 可用。
     *
     * Nav2 的 navigate_to_pose action server 在导航栈完全启动后才可用。
     * 本方法阻塞等待，直到 action server 响应或超时。
     *
     * @return action server 可用返回 true；超时（action_server_wait_timeout_sec_）返回 false。
     */
    bool wait_for_action_server();

    /**
     * @brief 发布初始位姿到 /initialpose 话题。
     *
     * 向 AMCL 发布机器人的初始位姿估计，使定位粒子收敛到正确位置。
     * 连续发布 5 次（间隔 200ms），因为 AMCL 可能在启动瞬间丢失消息。
     * 协方差矩阵对角线设为 0.25，表示对初始位置有 0.5 米的不确定性。
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
     *
     * @param[in] order_id 订单 ID。
     * @param[in] state 新的配送状态。
     * @param[in] station 当前关联的站点 ID（可选，默认空字符串）。
     * @param[in] progress 进度值，范围 [0.0, 1.0]（可选，默认 0.0）。
     * @param[in] error 错误信息（可选，默认空字符串，仅在失败时填充）。
     */
    void publish_status(const std::string & order_id, DeliveryState state,
                        const std::string & station = "",
                        float progress = 0.0f,
                        const std::string & error = "");

    /**
     * @brief 将内部状态枚举转换为 DeliveryStatus 消息中的 uint8 常量。
     *
     * DeliveryStatus.msg 中定义了 STATE_IDLE、STATE_GOING_TO_PICKUP 等常量，
     * 本方法负责将 C++ 枚举映射到这些 ROS 消息常量。
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
     */
    std::string state_to_string(DeliveryState state) const;

    // ====== 配送执行 ======

    /**
     * @brief 执行单个配送订单的完整流程。
     *
     * 按照 "导航取货点→等待装货→导航送货点→等待卸货" 四个阶段顺序执行。
     * 每个阶段开始前都会更新并发布状态，任一阶段失败则立即返回 false。
     *
     * @param[in,out] record 订单记录，执行过程中会更新其 state 和 error_msg 字段。
     * @return 全部四个阶段成功返回 true；任一阶段失败返回 false。
     *
     * @note 当前实现不支持重试。Phase 2 引入行为树后，重试逻辑将在 BT 层面实现。
     */
    bool execute_delivery(OrderRecord & record);

    /**
     * @brief 等待 /execute_delivery Action Server 可用。
     *
     * delivery_executor 启动后才会注册 Action Server，
     * 本方法阻塞等待其可用。
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
     *
     * @param[in] request 包含 DeliveryOrder 消息的请求。
     * @param[out] response accepted=true 表示接受，否则 reason 字段含拒绝原因。
     */
    void handle_submit_order(
        const std::shared_ptr<delivery_interfaces::srv::SubmitOrder::Request> request,
        std::shared_ptr<delivery_interfaces::srv::SubmitOrder::Response> response);

    /**
     * @brief CancelOrder 服务回调：取消队列中尚未执行的订单。
     *
     * 在订单队列中查找匹配的 order_id，找到则移除。
     * 已经开始执行或已完成的订单无法取消。
     *
     * @param[in] request 包含 order_id 的取消请求。
     * @param[out] response success=true 表示成功取消，否则 reason 字段含失败原因。
     */
    void handle_cancel_order(
        const std::shared_ptr<delivery_interfaces::srv::CancelOrder::Request> request,
        std::shared_ptr<delivery_interfaces::srv::CancelOrder::Response> response);

    /**
     * @brief GetDeliveryReport 服务回调：查询所有订单的配送状态报告。
     *
     * 返回排队中和已完成/失败的所有订单状态，用于外部系统监控。
     *
     * @param[in] request 空请求（无输入参数）。
     * @param[out] response reports 字段包含所有订单的 DeliveryStatus 列表。
     */
    void handle_get_report(
        const std::shared_ptr<delivery_interfaces::srv::GetDeliveryReport::Request> request,
        std::shared_ptr<delivery_interfaces::srv::GetDeliveryReport::Response> response);

    // ====== ROS 参数 ======
    std::string map_frame_;                     ///< 地图坐标系名称，默认 "map"，所有导航目标点都在此坐标系下
    std::string base_frame_;                    ///< 机器人本体坐标系名称，默认 "base_link"，用于 TF 查询
    std::string nav2_action_name_;              ///< Nav2 导航 Action 名称，默认 "navigate_to_pose"
    std::string station_config_path_;           ///< 站点配置 YAML 文件路径，由 launch 文件传入
    double clock_wait_timeout_sec_{30.0};       ///< 等待仿真时钟 /clock 的超时秒数
    double tf_wait_timeout_sec_{15.0};          ///< 等待 TF 链路 (map→base_link) 就绪的超时秒数
    double navigation_timeout_sec_{120.0};      ///< 单次导航任务的超时秒数（超时后取消导航）
    double wait_confirmation_timeout_sec_{60.0}; ///< 等待装/卸货确认的超时秒数
    double action_server_wait_timeout_sec_{10.0}; ///< 等待 Nav2 Action Server 上线的超时秒数
    double initial_x_{0.0};                     ///< 初始位姿 X 坐标（米），发布到 /initialpose
    double initial_y_{0.0};                     ///< 初始位姿 Y 坐标（米），发布到 /initialpose
    double initial_yaw_{0.0};                   ///< 初始位姿偏航角（弧度），发布到 /initialpose
    bool publish_initial_pose_{false};          ///< 是否在启动时发布初始位姿（仿真环境通常需要开启）
    bool use_sim_time_{true};                   ///< 是否使用仿真时钟（仿真环境设为 true，实机设为 false）

    // ====== 站点数据 ======
    std::unordered_map<std::string, Station> stations_;   ///< 站点 ID → Station 映射表，启动时从 YAML 加载

    // ====== 订单队列 ======
    std::deque<OrderRecord> order_queue_;                  ///< 待执行订单队列（按优先级排序，高优先级在前）
    std::vector<OrderRecord> completed_orders_;            ///< 已完成/失败订单的历史记录，用于报告查询
    std::mutex queue_mutex_;                               ///< 订单队列互斥锁，保护 order_queue_ 和 completed_orders_ 的并发访问

    // ====== ROS 通信接口 ======
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;   ///< Nav2 导航 Action 客户端，用于发送导航目标
    rclcpp::Publisher<DeliveryStatus>::SharedPtr status_pub_;           ///< 配送状态发布器，发布到 /delivery_status
    rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_; ///< 初始位姿发布器，发布到 /initialpose 供 AMCL 使用

    // ====== 服务端 ======
    rclcpp::Service<delivery_interfaces::srv::SubmitOrder>::SharedPtr submit_order_srv_;      ///< 订单提交服务端 (/submit_order)
    rclcpp::Service<delivery_interfaces::srv::CancelOrder>::SharedPtr cancel_order_srv_;      ///< 订单取消服务端 (/cancel_order)
    rclcpp::Service<delivery_interfaces::srv::GetDeliveryReport>::SharedPtr get_report_srv_;  ///< 配送报告查询服务端 (/get_delivery_report)

    // ====== 配送执行 Action Client ======
    rclcpp_action::Client<ExecuteDelivery>::SharedPtr delivery_action_client_; ///< ExecuteDelivery Action Client，调用 executor
    ExecuteDeliveryGoalHandle::SharedPtr current_goal_handle_;                 ///< 当前正在执行的配送 Goal 句柄（用于取消）
    std::mutex goal_handle_mutex_;                                             ///< 保护 current_goal_handle_ 的互斥锁

    // ====== 当前执行中的订单追踪 ======
    std::string current_order_id_;                                             ///< 当前正在执行的订单 ID（空字符串表示无执行中订单）
    std::optional<OrderRecord> current_order_;                                 ///< 当前正在执行的订单记录（用于报告查询）
    std::mutex current_order_mutex_;                                           ///< 保护 current_order_id_ 和 current_order_ 的互斥锁

    // ====== 回调组 ======
    /**
     * @brief Reentrant 回调组，将所有服务回调分配到此组。
     *
     * 使用 Reentrant 类型而非默认的 MutuallyExclusive 类型，原因是：
     * - 主线程的 run() 方法中调用了 rclcpp::sleep_for() 等阻塞操作
     * - 如果服务回调与主线程在同一 MutuallyExclusive 组中，会导致死锁
     * - Reentrant 组允许多个回调并发执行，确保订单提交/取消等操作不被阻塞
     */
    rclcpp::CallbackGroup::SharedPtr service_cb_group_;
};

}  // namespace delivery_core

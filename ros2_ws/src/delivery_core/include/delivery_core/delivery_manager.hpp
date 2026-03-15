#pragma once

/**
 * @file delivery_manager.hpp
 * @brief 配送管理节点声明。
 *
 * 负责接收配送订单、管理订单队列、协调导航与停靠确认流程。
 * 状态机: kIdle → kGoingToPickup → kWaitingLoad → kGoingToDropoff → kWaitingUnload → kComplete
 */

#include <deque>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "delivery_interfaces/msg/delivery_order.hpp"
#include "delivery_interfaces/msg/delivery_status.hpp"
#include "delivery_interfaces/msg/station_info.hpp"
#include "delivery_interfaces/srv/submit_order.hpp"
#include "delivery_interfaces/srv/cancel_order.hpp"
#include "delivery_interfaces/srv/get_delivery_report.hpp"

#include "yaml-cpp/yaml.h"

namespace delivery_core
{

/**
 * @brief 配送管理节点
 *
 * 核心职责：
 * 1. 从 YAML 加载站点配置
 * 2. 通过 SubmitOrder 服务接收订单，维护优先级队列
 * 3. 按序执行配送：导航取货点 → 等待装货确认 → 导航送货点 → 等待卸货确认
 * 4. 发布实时 DeliveryStatus 状态
 */
class DeliveryManager : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    using PoseStamped = geometry_msgs::msg::PoseStamped;
    using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
    using DeliveryOrder = delivery_interfaces::msg::DeliveryOrder;
    using DeliveryStatus = delivery_interfaces::msg::DeliveryStatus;
    using StationInfo = delivery_interfaces::msg::StationInfo;

    /**
     * @brief 构造函数，初始化参数、通信接口。
     */
    DeliveryManager();

    /**
     * @brief 主运行逻辑，阻塞执行配送循环。
     */
    void run();

private:
    /**
     * @brief 配送任务状态枚举
     */
    enum class DeliveryState
    {
        kIdle,             ///< 空闲，等待订单
        kGoingToPickup,    ///< 正在前往取货站点
        kWaitingLoad,      ///< 到达取货点，等待装货确认
        kGoingToDropoff,   ///< 正在前往送货站点
        kWaitingUnload,    ///< 到达送货点，等待卸货确认
        kComplete,         ///< 当前订单配送完成
        kFailed            ///< 当前订单配送失败
    };

    /**
     * @brief 2D 位姿，用于内部坐标存储
     */
    struct Pose2D
    {
        double x{0.0};     ///< X 坐标 (米)
        double y{0.0};     ///< Y 坐标 (米)
        double yaw{0.0};   ///< 偏航角 (弧度)
    };

    /**
     * @brief 站点描述，从 YAML 加载
     */
    struct Station
    {
        std::string id;       ///< 站点 ID
        Pose2D pose;          ///< 站点位姿
        uint8_t type{0};      ///< 站点类型 (0=pickup, 1=dropoff, 2=charge)
    };

    /**
     * @brief 订单记录，含状态追踪
     */
    struct OrderRecord
    {
        DeliveryOrder order;          ///< 原始订单
        DeliveryState state{DeliveryState::kIdle};  ///< 当前状态
        std::string error_msg;        ///< 错误信息
        rclcpp::Time start_time;      ///< 开始执行时间
    };

    // --- 配置加载 ---
    /**
     * @brief 从 YAML 文件加载站点配置。
     * @param[in] path YAML 文件路径。
     * @return 加载成功返回 true。
     */
    bool load_station_config(const std::string & path);

    // --- 环境准备 ---
    /**
     * @brief 等待仿真时钟可用。
     * @return 成功返回 true。
     */
    bool wait_for_time();

    /**
     * @brief 等待 TF 链路就绪 (map → base_link)。
     * @return 成功返回 true。
     */
    bool wait_for_tf();

    /**
     * @brief 等待 Nav2 action server 可用。
     * @return 成功返回 true。
     */
    bool wait_for_action_server();

    /**
     * @brief 发布初始位姿到 /initialpose。
     */
    void publish_initial_pose();

    // --- 导航 ---
    /**
     * @brief 构造 PoseStamped 消息。
     * @param[in] pose 2D 位姿。
     * @return ROS PoseStamped 消息。
     */
    PoseStamped make_pose(const Pose2D & pose) const;

    /**
     * @brief 导航到指定位姿。
     * @param[in] pose 目标位姿。
     * @param[in] label 日志标签。
     * @return 成功到达返回 true。
     */
    bool navigate_to(const Pose2D & pose, const std::string & label);

    // --- 状态管理 ---
    /**
     * @brief 更新并发布配送状态。
     * @param[in] order_id 订单 ID。
     * @param[in] state 新状态。
     * @param[in] station 关联站点 ID。
     * @param[in] progress 进度值。
     * @param[in] error 错误信息。
     */
    void publish_status(const std::string & order_id, DeliveryState state,
                        const std::string & station = "",
                        float progress = 0.0f,
                        const std::string & error = "");

    /**
     * @brief 将内部状态枚举转换为消息中的 uint8 常量。
     * @param[in] state 内部状态。
     * @return 消息状态值。
     */
    uint8_t state_to_msg(DeliveryState state) const;

    /**
     * @brief 将状态枚举转换为可读字符串。
     * @param[in] state 内部状态。
     * @return 状态字符串。
     */
    std::string state_to_string(DeliveryState state) const;

    // --- 配送执行 ---
    /**
     * @brief 执行单个配送订单的完整流程。
     * @param[in,out] record 订单记录。
     * @return 配送成功返回 true。
     */
    bool execute_delivery(OrderRecord & record);

    /**
     * @brief 等待外部装/卸货确认服务调用。
     * @param[in] confirm_service 确认服务名。
     * @param[in] order_id 订单 ID。
     * @param[in] station_id 站点 ID。
     * @param[in] state 当前状态 (kWaitingLoad 或 kWaitingUnload)。
     * @return 收到确认返回 true，超时或节点关闭返回 false。
     */
    bool wait_for_confirmation(const std::string & confirm_service,
                               const std::string & order_id,
                               const std::string & station_id,
                               DeliveryState state);

    // --- 服务回调 ---
    /**
     * @brief SubmitOrder 服务回调。
     */
    void handle_submit_order(
        const std::shared_ptr<delivery_interfaces::srv::SubmitOrder::Request> request,
        std::shared_ptr<delivery_interfaces::srv::SubmitOrder::Response> response);

    /**
     * @brief CancelOrder 服务回调。
     */
    void handle_cancel_order(
        const std::shared_ptr<delivery_interfaces::srv::CancelOrder::Request> request,
        std::shared_ptr<delivery_interfaces::srv::CancelOrder::Response> response);

    /**
     * @brief GetDeliveryReport 服务回调。
     */
    void handle_get_report(
        const std::shared_ptr<delivery_interfaces::srv::GetDeliveryReport::Request> request,
        std::shared_ptr<delivery_interfaces::srv::GetDeliveryReport::Response> response);

    // --- 参数 ---
    std::string map_frame_;                     ///< 地图坐标系
    std::string base_frame_;                    ///< 机器人坐标系
    std::string nav2_action_name_;              ///< Nav2 action 名称
    std::string station_config_path_;           ///< 站点配置文件路径
    double clock_wait_timeout_sec_{30.0};       ///< 时钟等待超时
    double tf_wait_timeout_sec_{15.0};          ///< TF 等待超时
    double navigation_timeout_sec_{120.0};      ///< 导航超时
    double wait_confirmation_timeout_sec_{60.0}; ///< 确认等待超时
    double action_server_wait_timeout_sec_{10.0}; ///< Action server 等待超时
    double initial_x_{0.0};                     ///< 初始位姿 X
    double initial_y_{0.0};                     ///< 初始位姿 Y
    double initial_yaw_{0.0};                   ///< 初始位姿 Yaw
    bool publish_initial_pose_{false};          ///< 是否发布初始位姿
    bool use_sim_time_{true};                   ///< 是否使用仿真时钟

    // --- 站点数据 ---
    std::unordered_map<std::string, Station> stations_;   ///< 站点 ID → Station 映射

    // --- 订单队列 ---
    std::deque<OrderRecord> order_queue_;                  ///< 待执行订单队列
    std::vector<OrderRecord> completed_orders_;            ///< 已完成订单记录
    std::mutex queue_mutex_;                               ///< 队列互斥锁

    // --- ROS 通信接口 ---
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;   ///< Nav2 action client
    rclcpp::Publisher<DeliveryStatus>::SharedPtr status_pub_;           ///< 状态发布器
    rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_; ///< 初始位姿发布器

    // --- 服务端 ---
    rclcpp::Service<delivery_interfaces::srv::SubmitOrder>::SharedPtr submit_order_srv_;
    rclcpp::Service<delivery_interfaces::srv::CancelOrder>::SharedPtr cancel_order_srv_;
    rclcpp::Service<delivery_interfaces::srv::GetDeliveryReport>::SharedPtr get_report_srv_;

    // --- 确认服务端（模拟人工确认） ---
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr confirm_load_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr confirm_unload_srv_;
    std::atomic<bool> load_confirmed_{false};     ///< 装货确认标志
    std::atomic<bool> unload_confirmed_{false};   ///< 卸货确认标志

    // --- 回调组（防止服务调用与导航回调互相阻塞） ---
    rclcpp::CallbackGroup::SharedPtr service_cb_group_;
};

}  // namespace delivery_core

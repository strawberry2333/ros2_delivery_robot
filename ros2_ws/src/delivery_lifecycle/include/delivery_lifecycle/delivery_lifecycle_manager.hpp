#pragma once

/**
 * @file delivery_lifecycle_manager.hpp
 * @brief 配送系统生命周期管理器声明。
 *
 * 管理配送系统节点的启动顺序：
 *   simulation → nav2 → delivery_executor → delivery_manager
 * 复用 Ros2Learning/lifecycle_manager 模式。
 */

#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>

namespace delivery_lifecycle
{

/**
 * @brief 配送系统生命周期管理器
 *
 * 通过标准 lifecycle 服务编排多个 LifecycleNode 的启动顺序。
 * 设计要点：
 *   - 服务客户端使用专用 CallbackGroup + 独立执行器线程
 *   - startup_sequence 在后台线程执行，不阻塞主 executor
 */
class DeliveryLifecycleManager : public rclcpp::Node
{
public:
    /**
     * @brief 构造管理器节点。
     * @param[in] options ROS 2 节点选项。
     */
    explicit DeliveryLifecycleManager(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

    /**
     * @brief 释放线程资源。
     */
    ~DeliveryLifecycleManager();

private:
    using ChangeState = lifecycle_msgs::srv::ChangeState;
    using GetState = lifecycle_msgs::srv::GetState;

    /**
     * @brief 被管理节点描述
     */
    struct ManagedNode
    {
        std::string name;  ///< 节点名
        rclcpp::Client<ChangeState>::SharedPtr change_state_client;
        rclcpp::Client<GetState>::SharedPtr get_state_client;
    };

    void startup_sequence();
    void load_managed_node_names(std::vector<std::string> & node_names);
    void initialize_managed_nodes(const std::vector<std::string> & node_names);
    void setup_service_executor();
    void start_background_threads();

    bool wait_for_all_services(std::chrono::seconds timeout);
    bool configure_all_nodes();
    void wait_transition_delay();
    bool activate_all_nodes();

    bool configure_node_if_needed(const ManagedNode & node);
    bool activate_node_if_needed(const ManagedNode & node);
    bool wait_for_service_ready(const ManagedNode & node, std::chrono::seconds timeout);
    bool change_state(const ManagedNode & node, uint8_t transition_id);
    uint8_t get_state(const ManagedNode & node);

    std::vector<ManagedNode> managed_nodes_;
    rclcpp::CallbackGroup::SharedPtr service_cb_group_;
    rclcpp::executors::SingleThreadedExecutor service_executor_;
    std::thread service_executor_thread_;
    std::thread startup_thread_;
};

}  // namespace delivery_lifecycle

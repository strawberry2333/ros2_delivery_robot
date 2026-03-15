#pragma once

/**
 * @file delivery_lifecycle_manager.hpp
 * @brief 配送系统生命周期管理器声明。
 *
 * 管理配送系统节点的启动顺序：
 *   simulation → nav2 → delivery_executor → delivery_manager
 * 复用 Ros2Learning/lifecycle_manager 模式。
 *
 * 设计背景：
 *   ROS 2 生命周期节点（LifecycleNode）支持 Unconfigured → Inactive → Active 的
 *   状态转换。本管理器按照 managed_nodes 参数中声明的顺序，依次对各节点执行
 *   configure 和 activate 操作，确保上游节点就绪后下游节点才启动，避免启动时序
 *   导致的服务不可用问题。
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
 *   - 服务客户端使用专用 CallbackGroup + 独立执行器线程，
 *     避免服务调用与主节点回调竞争同一执行器导致死锁
 *   - startup_sequence 在后台线程执行，不阻塞主 executor，
 *     使节点在启动编排期间仍能响应其他回调（如日志、参数查询等）
 *
 * 典型生命周期转换流程：
 *   Unconfigured --[configure]--> Inactive --[activate]--> Active
 */
class DeliveryLifecycleManager : public rclcpp::Node
{
public:
    /**
     * @brief 构造管理器节点。
     * @param[in] options ROS 2 节点选项，可用于传入参数覆盖、组件加载上下文等。
     *
     * 构造过程中完成以下初始化：
     *   1. 从参数服务器加载被管理节点名称列表
     *   2. 创建专用 CallbackGroup 和独立执行器
     *   3. 为每个被管理节点创建 change_state / get_state 服务客户端
     *   4. 启动后台线程执行节点编排序列
     */
    explicit DeliveryLifecycleManager(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

    /**
     * @brief 析构函数，释放线程资源。
     *
     * 等待 startup_thread_ 完成后，取消 service_executor_ 并等待其线程退出。
     * 注意：必须先 join startup_thread_，再 cancel executor，
     * 否则 startup_sequence 中的服务调用可能因执行器提前退出而挂起。
     */
    ~DeliveryLifecycleManager();

private:
    /// 生命周期状态转换服务类型别名
    using ChangeState = lifecycle_msgs::srv::ChangeState;
    /// 生命周期状态查询服务类型别名
    using GetState = lifecycle_msgs::srv::GetState;

    /**
     * @brief 被管理节点描述结构体
     *
     * 封装每个被管理 LifecycleNode 的名称及其对应的服务客户端，
     * 方便在遍历时统一操作。
     */
    struct ManagedNode
    {
        std::string name;  ///< 被管理节点的名称（需与节点实际注册名一致）
        rclcpp::Client<ChangeState>::SharedPtr change_state_client;  ///< change_state 服务客户端，用于触发状态转换
        rclcpp::Client<GetState>::SharedPtr get_state_client;  ///< get_state 服务客户端，用于查询当前生命周期状态
    };

    /**
     * @brief 启动编排主流程。
     *
     * 按顺序执行：等待服务就绪 → configure 所有节点 → 延时等待 → activate 所有节点。
     * 任一步骤失败则提前返回，不继续后续步骤。
     */
    void startup_sequence();

    /**
     * @brief 从 ROS 参数加载被管理节点名称列表。
     * @param[out] node_names 输出的节点名称列表。
     *
     * 同时声明 transition_delay_sec 参数（configure 与 activate 之间的等待时间）。
     */
    void load_managed_node_names(std::vector<std::string> & node_names);

    /**
     * @brief 为每个被管理节点创建服务客户端。
     * @param[in] node_names 节点名称列表。
     *
     * 服务端点格式为 /<node_name>/change_state 和 /<node_name>/get_state，
     * 这是 ROS 2 LifecycleNode 的标准服务命名约定。
     */
    void initialize_managed_nodes(const std::vector<std::string> & node_names);

    /**
     * @brief 创建专用 CallbackGroup 和独立执行器。
     *
     * 使用 MutuallyExclusive 类型的 CallbackGroup，确保同一时刻只有一个服务回调在执行。
     * 将该 CallbackGroup 绑定到独立的 SingleThreadedExecutor，使服务调用不与主执行器竞争。
     */
    void setup_service_executor();

    /**
     * @brief 启动后台线程。
     *
     * 启动两个线程：
     *   1. service_executor_thread_: 驱动服务客户端回调的独立执行器
     *   2. startup_thread_: 延时 2 秒后执行 startup_sequence，给各节点留出启动时间
     */
    void start_background_threads();

    /**
     * @brief 等待所有被管理节点的生命周期服务就绪。
     * @param[in] timeout 每个节点的等待超时时间。
     * @return 所有服务均就绪返回 true，任一超时返回 false。
     */
    bool wait_for_all_services(std::chrono::seconds timeout);

    /**
     * @brief 按顺序对所有节点执行 configure 转换。
     * @return 所有节点 configure 成功返回 true，任一失败返回 false。
     */
    bool configure_all_nodes();

    /**
     * @brief 在 configure 和 activate 之间等待指定延时。
     *
     * 延时时间由 transition_delay_sec 参数控制，给节点在 Inactive 状态下
     * 完成资源初始化（如订阅话题、加载地图等）留出缓冲时间。
     */
    void wait_transition_delay();

    /**
     * @brief 按顺序对所有节点执行 activate 转换。
     * @return 所有节点 activate 成功返回 true，任一失败返回 false。
     */
    bool activate_all_nodes();

    /**
     * @brief 对单个节点执行 configure（如果当前处于 Unconfigured 状态）。
     * @param[in] node 被管理节点描述。
     * @return configure 成功或节点不在 Unconfigured 状态（已跳过）返回 true，失败返回 false。
     *
     * 若节点不在 Unconfigured 状态，说明可能已被外部配置过，此时跳过并打印警告。
     */
    bool configure_node_if_needed(const ManagedNode & node);

    /**
     * @brief 对单个节点执行 activate（如果当前处于 Inactive 状态）。
     * @param[in] node 被管理节点描述。
     * @return activate 成功或节点不在 Inactive 状态（已跳过）返回 true，失败返回 false。
     *
     * 若节点不在 Inactive 状态，说明可能已被外部激活或尚未 configure，跳过并打印警告。
     */
    bool activate_node_if_needed(const ManagedNode & node);

    /**
     * @brief 等待单个节点的 change_state 和 get_state 服务可用。
     * @param[in] node 被管理节点描述。
     * @param[in] timeout 等待超时时间。
     * @return 两个服务均可用返回 true，任一超时返回 false。
     */
    bool wait_for_service_ready(const ManagedNode & node, std::chrono::seconds timeout);

    /**
     * @brief 调用 change_state 服务触发状态转换。
     * @param[in] node 被管理节点描述。
     * @param[in] transition_id 目标转换 ID（如 TRANSITION_CONFIGURE, TRANSITION_ACTIVATE）。
     * @return 服务调用成功且转换成功返回 true，超时或转换失败返回 false。
     *
     * 使用 5 秒超时等待服务响应，防止服务端异常导致管理器永久阻塞。
     */
    bool change_state(const ManagedNode & node, uint8_t transition_id);

    /**
     * @brief 查询节点当前生命周期状态。
     * @param[in] node 被管理节点描述。
     * @return 当前状态 ID（如 PRIMARY_STATE_UNCONFIGURED），超时返回 PRIMARY_STATE_UNKNOWN。
     */
    uint8_t get_state(const ManagedNode & node);

    /// 被管理节点列表，按参数中声明的顺序排列（启动编排按此顺序依次执行）
    std::vector<ManagedNode> managed_nodes_;

    /// 专用回调组，将服务客户端回调与主节点回调隔离，避免死锁
    rclcpp::CallbackGroup::SharedPtr service_cb_group_;

    /// 独立的单线程执行器，专门驱动 service_cb_group_ 中的服务客户端回调
    rclcpp::executors::SingleThreadedExecutor service_executor_;

    /// 执行器线程，运行 service_executor_.spin()，持续处理服务响应回调
    std::thread service_executor_thread_;

    /// 启动编排线程，延时后执行 startup_sequence()，不阻塞主执行器
    std::thread startup_thread_;
};

}  // namespace delivery_lifecycle

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
 *
 * 在这个项目里，它不是业务节点，而是“启动编排器”：
 *   - 它不接单，不导航，不做 BT 决策
 *   - 它只负责把 delivery_executor 这类 LifecycleNode 拉到正确状态
 *   - 它把“什么时候可以开始接单”这个系统级问题从业务代码里剥离出来
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
 * 该节点本身只是普通 rclcpp::Node，因此不会像 LifecycleNode 一样被系统自动
 * 推进状态，它的工作是远程调用其他节点的 lifecycle 服务。
 * 设计要点：
 *   - 服务客户端使用专用 CallbackGroup + 独立执行器线程，
 *     避免服务调用与主节点回调竞争同一执行器导致死锁
 *   - startup_sequence 在后台线程执行，不阻塞主 executor，
 *     使节点在启动编排期间仍能响应其他回调（如日志、参数查询等）
 *   - 节点启动顺序按 managed_nodes 参数严格执行，避免下游节点先于上游依赖就绪
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
     *
     * 构造函数不直接做生命周期转换，因为此时节点内部的服务客户端、
     * 执行器和线程依赖还没有完全建立，过早触发启动会让错误更难排查。
     */
    explicit DeliveryLifecycleManager(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

    /**
     * @brief 析构函数，释放线程资源。
     *
     * 等待 startup_thread_ 完成后，取消 service_executor_ 并等待其线程退出。
     * 注意：必须先 join startup_thread_，再 cancel executor，
     * 否则 startup_sequence 中的服务调用可能因执行器提前退出而挂起。
     *
     * 这个顺序很关键，因为 startup_thread_ 可能还在同步等待生命周期服务返回。
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
     *
     * 它的作用相当于系统总闸门：只有当下游节点的依赖都准备好，
     * 才允许整个配送系统进入可接单状态。
     */
    void startup_sequence();

    /**
     * @brief 从 ROS 参数加载被管理节点名称列表。
     * @param[out] node_names 输出的节点名称列表。
     *
     * 同时声明 transition_delay_sec 参数（configure 与 activate 之间的等待时间）。
     * 参数中的顺序就是编排顺序，因此 launch 文件里写什么顺序，实际就按什么顺序启动。
     */
    void load_managed_node_names(std::vector<std::string> & node_names);

    /**
     * @brief 为每个被管理节点创建服务客户端。
     * @param[in] node_names 节点名称列表。
     *
     * 服务端点格式为 /<node_name>/change_state 和 /<node_name>/get_state，
     * 这是 ROS 2 LifecycleNode 的标准服务命名约定。
     *
     * 这里不直接调用服务，而是先把客户端都建好，后续编排逻辑只负责按顺序触发。
     */
    void initialize_managed_nodes(const std::vector<std::string> & node_names);

    /**
     * @brief 创建专用 CallbackGroup 和独立执行器。
     *
     * 使用 MutuallyExclusive 类型的 CallbackGroup，确保同一时刻只有一个服务回调在执行。
     * 将该 CallbackGroup 绑定到独立的 SingleThreadedExecutor，使服务调用不与主执行器竞争。
     *
     * 这是本包最重要的并发隔离措施：
     *   - 主节点负责生命周期编排流程
     *   - service_executor_ 只负责处理生命周期服务客户端的异步响应
     *   - 两者分离后，startup_sequence 中的 wait_for 不会把自己所在的执行器堵死
     */
    void setup_service_executor();

    /**
     * @brief 启动后台线程。
     *
     * 启动两个线程：
     *   1. service_executor_thread_: 驱动服务客户端回调的独立执行器
     *   2. startup_thread_: 延时 2 秒后执行 startup_sequence，给各节点留出启动时间
     *
     * 这两个线程的职责不同：
     *   - 一个负责“把服务响应真正跑起来”
     *   - 一个负责“按顺序推进系统状态”
     *   分开后，服务请求可以阻塞等待响应，但不会阻塞响应本身的处理。
     */
    void start_background_threads();

    /**
     * @brief 等待所有被管理节点的生命周期服务就绪。
     * @param[in] timeout 每个节点的等待超时时间。
     * @return 所有服务均就绪返回 true，任一超时返回 false。
     *
     * 先等服务而不是直接发 configure，是因为 lifecycle 服务可能比节点本体晚一点
     * 注册；过早调用只会得到不稳定的启动失败。
     */
    bool wait_for_all_services(std::chrono::seconds timeout);

    /**
     * @brief 按顺序对所有节点执行 configure 转换。
     * @return 所有节点 configure 成功返回 true，任一失败返回 false。
     *
     * configure 阶段通常用于加载地图、站点配置、BT、导航参数等重资源。
     * 这些资源准备完成后，节点仍停留在 Inactive，避免提前对外提供业务能力。
     */
    bool configure_all_nodes();

    /**
     * @brief 在 configure 和 activate 之间等待指定延时。
     *
     * 延时时间由 transition_delay_sec 参数控制，给节点在 Inactive 状态下
     * 完成资源初始化（如订阅话题、加载地图等）留出缓冲时间。
     * 对 Nav2 一类节点来说，这段等待往往能显著降低“刚 configure 完就 activate，
     * 结果内部资源还没准备好”的启动抖动。
     */
    void wait_transition_delay();

    /**
     * @brief 按顺序对所有节点执行 activate 转换。
     * @return 所有节点 activate 成功返回 true，任一失败返回 false。
     *
     * activate 阶段表示节点已经准备好对外提供实际业务能力，例如开始接单、
     * 开始发布话题或开始处理 action 请求。
     */
    bool activate_all_nodes();

    /**
     * @brief 对单个节点执行 configure（如果当前处于 Unconfigured 状态）。
     * @param[in] node 被管理节点描述。
     * @return configure 成功或节点不在 Unconfigured 状态（已跳过）返回 true，失败返回 false。
     *
     * 若节点不在 Unconfigured 状态，说明可能已被外部配置过，此时跳过并打印警告。
     * 这种幂等式编排让管理器在重复启动或手动干预后更稳。
     */
    bool configure_node_if_needed(const ManagedNode & node);

    /**
     * @brief 对单个节点执行 activate（如果当前处于 Inactive 状态）。
     * @param[in] node 被管理节点描述。
     * @return activate 成功或节点不在 Inactive 状态（已跳过）返回 true，失败返回 false。
     *
     * 若节点不在 Inactive 状态，说明可能已被外部激活或尚未 configure，跳过并打印警告。
     * 这里同样保持幂等，避免重复激活把系统状态搞乱。
     */
    bool activate_node_if_needed(const ManagedNode & node);

    /**
     * @brief 等待单个节点的 change_state 和 get_state 服务可用。
     * @param[in] node 被管理节点描述。
     * @param[in] timeout 等待超时时间。
     * @return 两个服务均可用返回 true，任一超时返回 false。
     *
     * 只有两个服务都存在，编排器才能既“看状态”又“改状态”。
     */
    bool wait_for_service_ready(const ManagedNode & node, std::chrono::seconds timeout);

    /**
     * @brief 调用 change_state 服务触发状态转换。
     * @param[in] node 被管理节点描述。
     * @param[in] transition_id 目标转换 ID（如 TRANSITION_CONFIGURE, TRANSITION_ACTIVATE）。
     * @return 服务调用成功且转换成功返回 true，超时或转换失败返回 false。
     *
     * 使用 5 秒超时等待服务响应，防止服务端异常导致管理器永久阻塞。
     * 这里采用异步发送 + 同步等待，是为了把底层生命周期服务的复杂度封装起来，
     * 让上层编排逻辑仍然是顺序式代码。
     */
    bool change_state(const ManagedNode & node, uint8_t transition_id);

    /**
     * @brief 查询节点当前生命周期状态。
     * @param[in] node 被管理节点描述。
     * @return 当前状态 ID（如 PRIMARY_STATE_UNCONFIGURED），超时返回 PRIMARY_STATE_UNKNOWN。
     *
     * 先查状态再决定是否 configure/activate，可以让管理器在被外部手动操作后
     * 仍然保持一定容错性。
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

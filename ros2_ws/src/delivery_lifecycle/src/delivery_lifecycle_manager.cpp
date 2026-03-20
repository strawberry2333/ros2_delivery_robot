/**
 * @file delivery_lifecycle_manager.cpp
 * @brief 配送系统生命周期管理器实现。
 *
 * 复用 Ros2Learning/lifecycle_manager_node 模式，
 * 适配配送系统的节点启动编排需求。
 *
 * 核心思路：
 *   本管理器本身是一个普通 Node（非 LifecycleNode），通过服务客户端远程控制
 *   其他 LifecycleNode 的状态转换。使用独立执行器线程处理服务回调，避免与主
 *   执行器产生回调竞争和死锁问题。
 *
 * 这个节点的定位是“系统级启动编排器”：
 *   - 它不参与配送业务，只负责把 delivery_executor 之类的节点推到正确状态
 *   - 它把生命周期服务调用从业务逻辑里抽出去，让主业务节点更专注
 *   - 它通过后台线程和独立 executor 把同步等待的复杂性封装起来
 */

#include "delivery_lifecycle/delivery_lifecycle_manager.hpp"

#include <chrono>
#include <thread>

#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;
using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;

namespace delivery_lifecycle
{

/**
 * @brief 构造函数实现。
 *
 * 初始化流程严格按照依赖顺序执行：
 *   1. 先加载参数（获取需要管理的节点名称列表）
 *   2. 再创建执行器和回调组（服务客户端创建时需要指定回调组）
 *   3. 然后创建服务客户端（依赖回调组）
 *   4. 最后启动后台线程（依赖服务客户端和执行器均已就绪）
 *
 * 这种顺序不是形式主义，而是为了避免“线程先起、服务没准备好、状态却开始推进”
 * 的竞态问题。
 */
DeliveryLifecycleManager::DeliveryLifecycleManager(const rclcpp::NodeOptions & options)
    : rclcpp::Node("delivery_lifecycle_manager", options)
{
    // 步骤 1: 从参数服务器读取被管理节点的名称列表
    // 这个列表决定了后续编排的顺序，因此 launch 文件里的顺序是有业务意义的。
    std::vector<std::string> node_names;
    load_managed_node_names(node_names);

    // 步骤 2: 创建专用回调组和独立执行器，为后续服务客户端提供隔离的回调处理通道
    // 这里先把并发隔离搭好，再去创建服务客户端，避免后续回调与主线程混用同一执行器。
    setup_service_executor();

    // 步骤 3: 为每个被管理节点创建 change_state 和 get_state 服务客户端
    // 只创建客户端，不触发任何转换，实际启动动作留到 startup_sequence 统一控制。
    initialize_managed_nodes(node_names);

    // 步骤 4: 启动执行器线程和启动编排线程
    // 一个线程负责处理服务响应，一个线程负责推进编排流程，职责分离后更容易排查问题。
    start_background_threads();

    RCLCPP_INFO(get_logger(),
        "[DeliveryLifecycleManager] 将管理 %zu 个节点", managed_nodes_.size());
}

/**
 * @brief 析构函数实现。
 *
 * 线程清理顺序很重要：
 *   1. 先等待 startup_thread_ 完成，因为它可能正在调用服务
 *   2. 再取消 service_executor_，使其 spin() 退出
 *   3. 最后等待 service_executor_thread_ 退出
 * 如果顺序反转，startup_sequence 中的 future.wait_for() 可能永远无法完成。
 *
 * 这里的清理是为了保证退出时不会留下“后台线程还在等服务，但执行器已经停掉”的悬挂状态。
 */
DeliveryLifecycleManager::~DeliveryLifecycleManager()
{
    // 等待启动编排线程完成（它可能仍在进行服务调用）
    if (startup_thread_.joinable())
    {
        startup_thread_.join();
    }
    // 通知执行器停止 spin，使 service_executor_thread_ 可以退出
    service_executor_.cancel();
    // 等待执行器线程退出，确保服务客户端回调不再运行
    if (service_executor_thread_.joinable())
    {
        service_executor_thread_.join();
    }
}

/**
 * @brief 启动编排主流程实现。
 *
 * 按照 ROS 2 生命周期规范，节点必须先 configure 再 activate。
 * 中间插入延时是为了给节点在 Inactive 状态下完成内部初始化留出时间
 * （例如 nav2 节点在 configure 阶段加载地图，需要一定时间）。
 *
 * 编排采用"快速失败"策略：任一步骤出错立即返回，不继续后续操作，
 * 避免在依赖节点未就绪的情况下激活下游节点导致运行时错误。
 *
 * 对外看，这个函数的意义就是：当它返回成功时，系统才真正进入“可接单”的阶段。
 */
void DeliveryLifecycleManager::startup_sequence()
{
    // 第一步：等待所有被管理节点的生命周期服务就绪（超时 10 秒）
    // 先确认服务存在，再做状态转换，避免服务未注册时发送无效请求。
    if (!wait_for_all_services(10s))
    {
        return;
    }
    // 第二步：按顺序将所有节点从 Unconfigured 转为 Inactive
    // configure 阶段通常会加载地图、站点配置或其他重资源。
    if (!configure_all_nodes())
    {
        return;
    }
    // 第三步：等待一段时间，让节点在 Inactive 状态下完成内部资源初始化
    // 这段缓冲时间能降低“刚 configure 完就 activate，但内部还没准备好”的启动抖动。
    wait_transition_delay();
    // 第四步：按顺序将所有节点从 Inactive 转为 Active
    // 到这里才意味着节点开始真正对外提供业务能力。
    if (!activate_all_nodes())
    {
        return;
    }

    RCLCPP_INFO(get_logger(),
        "[DeliveryLifecycleManager] 所有节点已 Active");
}

/**
 * @brief 从参数服务器加载被管理节点名称。
 *
 * 声明两个参数：
 *   - managed_nodes: 字符串数组，列出需要管理的节点名（按启动顺序）
 *   - transition_delay_sec: 浮点数，configure 与 activate 之间的等待秒数
 *
 * 参数默认值为空数组和 2.0 秒，可通过 launch 文件或命令行覆盖。
 *
 * 其中 managed_nodes 的顺序很重要，它就是编排顺序本身。
 */
void DeliveryLifecycleManager::load_managed_node_names(
    std::vector<std::string> & node_names)
{
    // 声明 managed_nodes 参数，默认为空数组（实际使用时需通过 launch 文件传入）
    this->declare_parameter<std::vector<std::string>>(
        "managed_nodes", std::vector<std::string>{});
    // 声明 transition_delay_sec 参数，控制 configure → activate 之间的等待时间
    // 这不是业务逻辑延迟，而是给被管理节点留出内部初始化缓冲。
    this->declare_parameter("transition_delay_sec", 2.0);
    // 读取参数值
    node_names = this->get_parameter("managed_nodes").as_string_array();
}

/**
 * @brief 为每个被管理节点创建服务客户端。
 *
 * ROS 2 LifecycleNode 自动提供以下标准服务：
 *   - /<node_name>/change_state: 触发状态转换
 *   - /<node_name>/get_state: 查询当前状态
 *
 * 所有服务客户端都绑定到 service_cb_group_，确保其回调由独立执行器处理，
 * 而不是主节点的默认执行器。这是避免死锁的关键设计。
 *
 * 这里既建立“看状态”的通道，也建立“改状态”的通道，后续编排过程才有闭环。
 */
void DeliveryLifecycleManager::initialize_managed_nodes(
    const std::vector<std::string> & node_names)
{
    for (const auto & name : node_names)
    {
        ManagedNode node;
        node.name = name;
        // 创建 change_state 服务客户端，指定专用回调组
        // 这个客户端负责触发 configure / activate 等生命周期转换。
        node.change_state_client = this->create_client<ChangeState>(
            "/" + name + "/change_state",
            rclcpp::ServicesQoS(), service_cb_group_);
        // 创建 get_state 服务客户端，同样使用专用回调组
        // 编排器会先查状态，再决定是否需要执行转换，保持幂等。
        node.get_state_client = this->create_client<GetState>(
            "/" + name + "/get_state",
            rclcpp::ServicesQoS(), service_cb_group_);
        managed_nodes_.push_back(std::move(node));
    }
}

/**
 * @brief 创建专用回调组和独立执行器。
 *
 * 为什么需要独立执行器：
 *   如果服务客户端的回调与本节点共享同一个执行器，当在回调中同步等待
 *   服务响应（future.wait_for）时，执行器线程被阻塞，无法处理服务响应
 *   回调，导致死锁。使用独立执行器可以完全避免这个问题。
 *
 * MutuallyExclusive 回调组保证同一时刻只有一个服务回调在执行，
 * 简化了并发控制，且对本场景而言不影响性能（服务调用本身是串行的）。
 *
 * 这一步实际上是在给“同步写法”铺路：上层代码可以继续按顺序写，
 * 但底层响应处理被放到另一条执行链上。
 */
void DeliveryLifecycleManager::setup_service_executor()
{
    // 创建互斥回调组，同一时刻只允许一个回调执行
    service_cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    // 将回调组注册到独立执行器，使其回调由 service_executor_ 驱动
    // 这样 future.wait_for 等同步等待才不会把自己等待的响应堵在同一个执行器里。
    service_executor_.add_callback_group(
        service_cb_group_, this->get_node_base_interface());
}

/**
 * @brief 启动后台线程。
 *
 * service_executor_thread_: 持续 spin 独立执行器，处理服务客户端回调。
 *   该线程在整个节点生命周期内运行，直到析构时被 cancel + join。
 *
 * startup_thread_: 延时 2 秒后启动编排序列。
 *   延时的目的是给其他节点留出启动和注册服务的时间。如果立即执行，
 *   被管理节点可能还未完成初始化，其生命周期服务尚未可用。
 *
 * 两个线程的分工很明确：一个“跑服务响应”，一个“推进状态机”。
 */
void DeliveryLifecycleManager::start_background_threads()
{
    // 启动独立执行器线程，持续处理服务客户端的响应回调
    service_executor_thread_ = std::thread([this]() {
        service_executor_.spin();
    });

    // 启动编排线程，延时 2 秒后开始节点编排
    // 延时是为了等待被管理节点完成初始化并注册生命周期服务。
    startup_thread_ = std::thread([this]() {
        std::this_thread::sleep_for(2s);
        startup_sequence();
    });
}

/**
 * @brief 等待所有被管理节点的服务就绪。
 *
 * 按 managed_nodes_ 列表顺序逐一等待，某个节点超时则整体失败。
 * 这保证了在开始状态转换前，所有目标节点都已正常启动并注册了生命周期服务。
 *
 * 这一步先做“可达性确认”，避免后续 configure / activate 请求发出去却没有服务端响应。
 */
bool DeliveryLifecycleManager::wait_for_all_services(std::chrono::seconds timeout)
{
    for (const auto & node : managed_nodes_)
    {
        RCLCPP_INFO(get_logger(),
            "[DeliveryLifecycleManager] 等待节点 '%s' 服务...", node.name.c_str());
        if (!wait_for_service_ready(node, timeout))
        {
            RCLCPP_ERROR(get_logger(),
                "[DeliveryLifecycleManager] '%s' 服务超时", node.name.c_str());
            return false;
        }
    }
    return true;
}

/**
 * @brief 按顺序 configure 所有节点。
 *
 * 按列表顺序执行，确保上游节点先完成 configure。
 * 例如：导航节点需要先于配送管理器完成配置，因为配送管理器在 configure
 * 阶段可能需要查询导航节点提供的服务。
 *
 * configure 通常意味着加载配置、创建资源、建立订阅，但还不允许对外进入 Active。
 */
bool DeliveryLifecycleManager::configure_all_nodes()
{
    RCLCPP_INFO(get_logger(), "[DeliveryLifecycleManager] CONFIGURE ALL");
    for (const auto & node : managed_nodes_)
    {
        if (!configure_node_if_needed(node))
        {
            return false;
        }
    }
    return true;
}

/**
 * @brief 在 configure 和 activate 之间插入延时等待。
 *
 * 延时时间从 transition_delay_sec 参数读取。
 * 此延时的作用是给节点在 Inactive 状态下完成 on_configure 回调中的
 * 异步初始化操作留出时间，例如：
 *   - nav2 节点加载地图文件
 *   - 传感器节点建立硬件连接
 *   - 配送管理器加载站点配置
 *
 * 这里的等待不是“为了慢一点”，而是为了给外部依赖完成启动收尾。
 */
void DeliveryLifecycleManager::wait_transition_delay()
{
    const double delay_sec =
        this->get_parameter("transition_delay_sec").as_double();
    RCLCPP_INFO(get_logger(),
        "[DeliveryLifecycleManager] 等待 %.1f 秒后 activate...", delay_sec);
    // 将浮点秒数转为毫秒精度的 duration，避免浮点数直接构造 chrono::seconds 的精度丢失
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int64_t>(delay_sec * 1000.0)));
}

/**
 * @brief 按顺序 activate 所有节点。
 *
 * 同样按列表顺序执行，确保上游节点先进入 Active 状态。
 * 激活后节点开始正常处理业务逻辑（发布话题、处理订阅等）。
 *
 * 对 delivery_executor 来说，Active 意味着它已经可以真正开始接收 action goal。
 */
bool DeliveryLifecycleManager::activate_all_nodes()
{
    RCLCPP_INFO(get_logger(), "[DeliveryLifecycleManager] ACTIVATE ALL");
    for (const auto & node : managed_nodes_)
    {
        if (!activate_node_if_needed(node))
        {
            return false;
        }
    }
    return true;
}

/**
 * @brief 对单个节点执行条件化 configure。
 *
 * 先查询节点当前状态，只有处于 Unconfigured 状态时才发送 configure 转换请求。
 * 如果节点已经不在 Unconfigured 状态（可能被其他管理器或手动操作过），
 * 则跳过并返回成功，不视为错误——这提高了系统的容错性。
 *
 * 这让管理器在重复启动时更稳，不需要假设系统总是从零开始。
 */
bool DeliveryLifecycleManager::configure_node_if_needed(const ManagedNode & node)
{
    // 先查询当前状态，避免对非 Unconfigured 节点发送无效的 configure 请求
    const uint8_t current_state = get_state(node);
    RCLCPP_INFO(get_logger(),
        "[DeliveryLifecycleManager] '%s' 状态=%u，发送 configure...",
        node.name.c_str(), current_state);

    // 如果节点不在 Unconfigured 状态，跳过（可能已被外部配置过）
    if (current_state != State::PRIMARY_STATE_UNCONFIGURED)
    {
        RCLCPP_WARN(get_logger(),
            "[DeliveryLifecycleManager] '%s' 不在 Unconfigured，跳过",
            node.name.c_str());
        return true;
    }

    // 发送 configure 转换请求
    if (!change_state(node, Transition::TRANSITION_CONFIGURE))
    {
        RCLCPP_ERROR(get_logger(),
            "[DeliveryLifecycleManager] '%s' configure 失败", node.name.c_str());
        return false;
    }

    RCLCPP_INFO(get_logger(),
        "[DeliveryLifecycleManager] '%s' configure 完成", node.name.c_str());
    return true;
}

/**
 * @brief 对单个节点执行条件化 activate。
 *
 * 逻辑与 configure_node_if_needed 类似：先查询状态，只有处于 Inactive 时才激活。
 * 如果节点已在 Active 或其他状态，跳过并返回成功。
 *
 * 这种处理方式比“无条件激活”更安全，因为它允许外部手动介入后继续运行。
 */
bool DeliveryLifecycleManager::activate_node_if_needed(const ManagedNode & node)
{
    // 先查询当前状态，确保节点处于 Inactive 状态（即已完成 configure）
    const uint8_t current_state = get_state(node);
    RCLCPP_INFO(get_logger(),
        "[DeliveryLifecycleManager] '%s' 状态=%u，发送 activate...",
        node.name.c_str(), current_state);

    // 如果节点不在 Inactive 状态，跳过（可能尚未 configure 或已被激活）
    if (current_state != State::PRIMARY_STATE_INACTIVE)
    {
        RCLCPP_WARN(get_logger(),
            "[DeliveryLifecycleManager] '%s' 不在 Inactive，跳过",
            node.name.c_str());
        return true;
    }

    // 发送 activate 转换请求
    if (!change_state(node, Transition::TRANSITION_ACTIVATE))
    {
        RCLCPP_ERROR(get_logger(),
            "[DeliveryLifecycleManager] '%s' activate 失败", node.name.c_str());
        return false;
    }

    RCLCPP_INFO(get_logger(),
        "[DeliveryLifecycleManager] '%s' activate 完成", node.name.c_str());
    return true;
}

/**
 * @brief 等待单个节点的两个生命周期服务可用。
 *
 * 分别等待 change_state 和 get_state 服务。两者都必须可用才算成功。
 * 注意：每个服务都独立计算超时时间，最坏情况下总等待时间为 2 * timeout。
 *
 * 这一步相当于确认被管理节点已经把 lifecycle 接口挂出来了。
 */
bool DeliveryLifecycleManager::wait_for_service_ready(
    const ManagedNode & node, std::chrono::seconds timeout)
{
    // 先等待 change_state 服务（用于触发状态转换）
    if (!node.change_state_client->wait_for_service(timeout))
    {
        return false;
    }
    // 再等待 get_state 服务（用于查询当前状态）
    if (!node.get_state_client->wait_for_service(timeout))
    {
        return false;
    }
    return true;
}

/**
 * @brief 调用 change_state 服务执行状态转换。
 *
 * 使用异步发送 + 同步等待模式：async_send_request 发送请求后，
 * 通过 future.wait_for 等待响应，设置 5 秒超时防止永久阻塞。
 *
 * 注意：这里的 future.wait_for 能正常工作，是因为服务回调由独立的
 * service_executor_ 处理。如果使用主执行器，在回调中 wait_for 会死锁。
 *
 * 从调用者角度看它是同步函数，但内部靠独立执行器保证不会堵死自己。
 */
bool DeliveryLifecycleManager::change_state(
    const ManagedNode & node, uint8_t transition_id)
{
    // 构造请求，设置目标转换 ID
    auto request = std::make_shared<ChangeState::Request>();
    request->transition.id = transition_id;

    // 异步发送请求，获取 future 用于等待响应
    auto future = node.change_state_client->async_send_request(request);
    // 同步等待响应，60 秒超时
    // executor 的 on_activate 包含 Nav2 就绪等待（最多 30s），需留足余量。
    if (future.wait_for(60s) != std::future_status::ready)
    {
        RCLCPP_ERROR(get_logger(),
            "[DeliveryLifecycleManager] change_state(%u) 超时: %s",
            transition_id, node.name.c_str());
        return false;
    }
    // 返回服务端报告的转换是否成功
    return future.get()->success;
}

/**
 * @brief 查询节点当前生命周期状态。
 *
 * 与 change_state 类似，使用异步发送 + 同步等待模式。
 * 超时时返回 PRIMARY_STATE_UNKNOWN，调用方应据此判断并做错误处理。
 *
 * 这里的状态查询是编排器做“分支判断”的依据。
 */
uint8_t DeliveryLifecycleManager::get_state(const ManagedNode & node)
{
    auto request = std::make_shared<GetState::Request>();
    auto future = node.get_state_client->async_send_request(request);
    // 同步等待响应，5 秒超时
    if (future.wait_for(5s) != std::future_status::ready)
    {
        RCLCPP_ERROR(get_logger(),
            "[DeliveryLifecycleManager] get_state 超时: %s", node.name.c_str());
        // 超时返回 UNKNOWN 状态，调用方会据此做相应处理
        return State::PRIMARY_STATE_UNKNOWN;
    }
    // 从响应中提取当前状态 ID
    return future.get()->current_state.id;
}

}  // namespace delivery_lifecycle

// 注册为 ROS 2 组件节点，支持通过 composition 模式动态加载
// 这使得本节点可以在运行时被 ComponentManager 加载到共享进程中，
// 实现零拷贝通信和更高效的资源利用。
RCLCPP_COMPONENTS_REGISTER_NODE(delivery_lifecycle::DeliveryLifecycleManager)

/**
 * @file delivery_manager.cpp
 * @brief 配送管理节点实现。
 *
 * 从 Ros2Learning/task_runner 重构而来，将"导航→抓取→导航→放置"流程
 * 改为"导航取货点→等待装货确认→导航送货点→等待卸货确认"的真实配送模式。
 *
 * 本文件包含 DeliveryManager 类的全部方法实现：
 * - 构造函数：参数声明与通信接口初始化
 * - run()：主配送循环
 * - execute_delivery()：单订单配送流程
 * - 环境准备：时钟、TF、Action Server 等待
 * - 导航：Nav2 Action 调用
 * - 服务回调：订单提交、取消、报告
 * - 状态管理：状态发布与转换
 */

#include "delivery_core/delivery_manager.hpp"

#include <tf2/LinearMath/Quaternion.h>            // 四元数运算（欧拉角→四元数转换）
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // tf2 与 geometry_msgs 之间的类型转换
#include <tf2_ros/buffer.h>                        // TF 缓冲区
#include <tf2_ros/transform_listener.h>            // TF 监听器

#include <chrono>

using namespace std::chrono_literals;  // 启用 5s, 200ms 等时间字面量

namespace delivery_core
{

/**
 * @brief 构造函数实现：声明参数、创建通信接口。
 *
 * 初始化顺序很重要：
 * 1. 先声明参数（后续创建通信接口时可能依赖参数值）
 * 2. 创建回调组（服务创建时需要指定回调组）
 * 3. 创建 Action Client、Publisher、Service Server
 */
DeliveryManager::DeliveryManager()
    : Node("delivery_manager")
{
    // === 声明参数 ===
    // 坐标系配置：通常不需要修改，除非使用自定义命名空间
    map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
    nav2_action_name_ = this->declare_parameter<std::string>(
        "nav2_action_name", "navigate_to_pose");
    // 站点配置路径由 launch 文件通过参数传入
    station_config_path_ = this->declare_parameter<std::string>("station_config", "");

    // 各阶段的超时配置，可通过 YAML 参数文件调整
    clock_wait_timeout_sec_ = this->declare_parameter<double>(
        "clock_wait_timeout_sec", 30.0);
    tf_wait_timeout_sec_ = this->declare_parameter<double>(
        "tf_wait_timeout_sec", 15.0);
    navigation_timeout_sec_ = this->declare_parameter<double>(
        "navigation_timeout_sec", 120.0);
    wait_confirmation_timeout_sec_ = this->declare_parameter<double>(
        "wait_confirmation_timeout_sec", 60.0);
    action_server_wait_timeout_sec_ = this->declare_parameter<double>(
        "action_server_wait_timeout_sec", 10.0);

    // 初始位姿参数：用于 AMCL 定位初始化
    initial_x_ = this->declare_parameter<double>("initial_x", 0.0);
    initial_y_ = this->declare_parameter<double>("initial_y", 0.0);
    initial_yaw_ = this->declare_parameter<double>("initial_yaw", 0.0);
    publish_initial_pose_ = this->declare_parameter<bool>("publish_initial_pose", true);

    // use_sim_time 可能已由 launch 文件预设，需要先检查是否存在
    // 避免重复声明导致异常
    if (this->has_parameter("use_sim_time"))
    {
        this->get_parameter("use_sim_time", use_sim_time_);
    }
    else
    {
        use_sim_time_ = this->declare_parameter<bool>("use_sim_time", true);
    }

    // === 创建回调组 ===
    // 使用 Reentrant 类型，允许服务回调并发执行，
    // 避免与主线程的阻塞操作产生死锁
    service_cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::Reentrant);

    // === 创建通信接口 ===
    // Nav2 导航 Action 客户端
    action_client_ = rclcpp_action::create_client<NavigateToPose>(
        this, nav2_action_name_);

    // 配送状态发布器（QoS depth=10，保留最近 10 条消息）
    status_pub_ = this->create_publisher<DeliveryStatus>("delivery_status", 10);
    // 初始位姿发布器，发布到全局话题 /initialpose（AMCL 订阅此话题）
    initial_pose_pub_ = this->create_publisher<PoseWithCovarianceStamped>(
        "/initialpose", 10);

    // === 创建服务端 ===
    // 所有服务都绑定到 service_cb_group_，确保不会被主线程阻塞
    submit_order_srv_ = this->create_service<delivery_interfaces::srv::SubmitOrder>(
        "submit_order",
        std::bind(&DeliveryManager::handle_submit_order, this,
                  std::placeholders::_1, std::placeholders::_2),
        rclcpp::ServicesQoS(), service_cb_group_);

    cancel_order_srv_ = this->create_service<delivery_interfaces::srv::CancelOrder>(
        "cancel_order",
        std::bind(&DeliveryManager::handle_cancel_order, this,
                  std::placeholders::_1, std::placeholders::_2),
        rclcpp::ServicesQoS(), service_cb_group_);

    get_report_srv_ = this->create_service<delivery_interfaces::srv::GetDeliveryReport>(
        "get_delivery_report",
        std::bind(&DeliveryManager::handle_get_report, this,
                  std::placeholders::_1, std::placeholders::_2),
        rclcpp::ServicesQoS(), service_cb_group_);

    // === 创建装/卸货确认服务端 ===
    // 使用 lambda 回调而非成员函数绑定，因为逻辑简单（仅设置原子标志）
    // memory_order_release 确保标志的写入对其他线程立即可见
    confirm_load_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "confirm_load",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
            load_confirmed_.store(true, std::memory_order_release);
            response->success = true;
            response->message = "装货确认已接收";
            RCLCPP_INFO(get_logger(), "收到装货确认信号");
        },
        rclcpp::ServicesQoS(), service_cb_group_);

    confirm_unload_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "confirm_unload",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
            unload_confirmed_.store(true, std::memory_order_release);
            response->success = true;
            response->message = "卸货确认已接收";
            RCLCPP_INFO(get_logger(), "收到卸货确认信号");
        },
        rclcpp::ServicesQoS(), service_cb_group_);

    RCLCPP_INFO(get_logger(), "DeliveryManager 节点已初始化");
}

/**
 * @brief 主运行逻辑实现。
 *
 * 采用"准备→循环"两阶段模式：
 * - 准备阶段严格按顺序执行，任一步骤失败则直接返回（无法配送）
 * - 主循环以 2Hz 轮询订单队列，这个频率在响应速度和 CPU 占用之间取得平衡
 */
void DeliveryManager::run()
{
    // --- 准备阶段 1: 仿真时钟 ---
    // 在 use_sim_time 模式下必须先等待 /clock，否则 this->now() 始终返回 0
    if (use_sim_time_)
    {
        RCLCPP_INFO(get_logger(), "等待仿真时钟 /clock...");
        if (!wait_for_time())
        {
            RCLCPP_ERROR(get_logger(), "等待时钟超时");
            return;
        }
    }

    // --- 准备阶段 2: 加载站点配置 ---
    // 站点配置是核心数据，没有站点信息则无法验证和执行任何订单
    if (!load_station_config(station_config_path_))
    {
        RCLCPP_ERROR(get_logger(), "站点配置加载失败");
        return;
    }

    // --- 准备阶段 3: 等待导航服务 ---
    // Nav2 导航栈启动较慢，需要等待其 action server 就绪
    if (!wait_for_action_server())
    {
        RCLCPP_ERROR(get_logger(), "Nav2 action server 不可用");
        return;
    }

    // 发布初始位姿，帮助 AMCL 快速收敛定位
    if (publish_initial_pose_)
    {
        publish_initial_pose();
    }

    // 等待 TF 链路就绪（map→base_link），确认定位系统正常工作
    if (!wait_for_tf())
    {
        RCLCPP_ERROR(get_logger(), "TF 链路不可用");
        return;
    }

    RCLCPP_INFO(get_logger(),
        "系统就绪，已加载 %zu 个站点。等待订单提交...", stations_.size());

    // --- 主循环：从队列取订单并执行 ---
    // 使用 2Hz 轮询而非条件变量，原因是实现简单且对配送场景而言延迟可接受
    rclcpp::Rate rate(2.0);
    while (rclcpp::ok())
    {
        OrderRecord current_order;
        bool has_order = false;

        // 使用最小化的锁范围：仅在访问共享队列时加锁，
        // 避免在执行配送（可能耗时数分钟）期间持有锁
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            if (!order_queue_.empty())
            {
                // 取出队头订单（优先级最高的），拷贝后释放锁
                current_order = order_queue_.front();
                order_queue_.pop_front();
                has_order = true;
            }
        }

        if (has_order)
        {
            // 记录开始时间，用于后续统计配送耗时
            current_order.start_time = this->now();
            RCLCPP_INFO(get_logger(), "开始执行订单 [%s]: %s → %s",
                        current_order.order.order_id.c_str(),
                        current_order.order.pickup_station.c_str(),
                        current_order.order.dropoff_station.c_str());

            // 执行配送全流程（阻塞，可能耗时数分钟）
            bool success = execute_delivery(current_order);

            // 将完成的订单存入历史记录（无论成功还是失败）
            {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                completed_orders_.push_back(current_order);
            }

            if (success)
            {
                RCLCPP_INFO(get_logger(), "订单 [%s] 配送完成",
                            current_order.order.order_id.c_str());
            }
            else
            {
                RCLCPP_WARN(get_logger(), "订单 [%s] 配送失败: %s",
                            current_order.order.order_id.c_str(),
                            current_order.error_msg.c_str());
            }
        }

        // 没有订单时 sleep，减少 CPU 空转
        rate.sleep();
    }
}

/**
 * @brief 执行单个配送订单的完整流程。
 *
 * 四阶段线性流程，每个阶段的进度值设定：
 * - 0.1: 开始前往取货点
 * - 0.3: 到达取货点，等待装货
 * - 0.5: 开始前往送货点
 * - 0.8: 到达送货点，等待卸货
 * - 1.0: 配送完成
 * 进度值的分配反映了实际时间占比的预估。
 */
bool DeliveryManager::execute_delivery(OrderRecord & record)
{
    const auto & order = record.order;

    // 查找取货和送货站点——虽然 handle_submit_order 中已做过验证，
    // 这里再次检查是为了防御性编程（配置可能在运行中被意外修改）
    auto pickup_it = stations_.find(order.pickup_station);
    auto dropoff_it = stations_.find(order.dropoff_station);

    if (pickup_it == stations_.end())
    {
        record.state = DeliveryState::kFailed;
        record.error_msg = "取货站点不存在: " + order.pickup_station;
        publish_status(order.order_id, DeliveryState::kFailed, "", 0.0f,
                       record.error_msg);
        return false;
    }
    if (dropoff_it == stations_.end())
    {
        record.state = DeliveryState::kFailed;
        record.error_msg = "送货站点不存在: " + order.dropoff_station;
        publish_status(order.order_id, DeliveryState::kFailed, "", 0.0f,
                       record.error_msg);
        return false;
    }

    const auto & pickup = pickup_it->second;
    const auto & dropoff = dropoff_it->second;

    // [阶段 1] 导航到取货点
    // 先更新状态再执行导航，使外部监控能实时追踪
    record.state = DeliveryState::kGoingToPickup;
    publish_status(order.order_id, DeliveryState::kGoingToPickup,
                   pickup.id, 0.1f);

    if (!navigate_to(pickup.pose, "取货点 " + pickup.id))
    {
        record.state = DeliveryState::kFailed;
        record.error_msg = "导航到取货点失败";
        publish_status(order.order_id, DeliveryState::kFailed,
                       pickup.id, 0.0f, record.error_msg);
        return false;
    }

    // [阶段 2] 等待装货确认
    // 机器人到达取货点后停靠等待，人工通过 /confirm_load 服务触发确认
    record.state = DeliveryState::kWaitingLoad;
    publish_status(order.order_id, DeliveryState::kWaitingLoad,
                   pickup.id, 0.3f);

    if (!wait_for_confirmation("confirm_load", order.order_id,
                               pickup.id, DeliveryState::kWaitingLoad))
    {
        record.state = DeliveryState::kFailed;
        record.error_msg = "等待装货确认超时";
        publish_status(order.order_id, DeliveryState::kFailed,
                       pickup.id, 0.0f, record.error_msg);
        return false;
    }

    // [阶段 3] 导航到送货点
    record.state = DeliveryState::kGoingToDropoff;
    publish_status(order.order_id, DeliveryState::kGoingToDropoff,
                   dropoff.id, 0.5f);

    if (!navigate_to(dropoff.pose, "送货点 " + dropoff.id))
    {
        record.state = DeliveryState::kFailed;
        record.error_msg = "导航到送货点失败";
        publish_status(order.order_id, DeliveryState::kFailed,
                       dropoff.id, 0.0f, record.error_msg);
        return false;
    }

    // [阶段 4] 等待卸货确认
    // 机器人到达送货点后停靠等待，人工通过 /confirm_unload 服务触发确认
    record.state = DeliveryState::kWaitingUnload;
    publish_status(order.order_id, DeliveryState::kWaitingUnload,
                   dropoff.id, 0.8f);

    if (!wait_for_confirmation("confirm_unload", order.order_id,
                               dropoff.id, DeliveryState::kWaitingUnload))
    {
        record.state = DeliveryState::kFailed;
        record.error_msg = "等待卸货确认超时";
        publish_status(order.order_id, DeliveryState::kFailed,
                       dropoff.id, 0.0f, record.error_msg);
        return false;
    }

    // [完成] 所有阶段成功
    record.state = DeliveryState::kComplete;
    publish_status(order.order_id, DeliveryState::kComplete,
                   dropoff.id, 1.0f);
    return true;
}

/**
 * @brief 等待装/卸货确认的实现。
 *
 * 采用轮询（polling）模式而非条件变量（condition_variable），原因是：
 * 1. 确认信号来自 ROS 服务回调（跨回调组），条件变量需要额外的同步机制
 * 2. 轮询间隔 500ms 对用户操作场景完全可接受
 * 3. 实现简单，不易出错
 */
bool DeliveryManager::wait_for_confirmation(
    const std::string & confirm_service,
    const std::string & order_id,
    const std::string & station_id,
    DeliveryState state)
{
    // 根据等待类型选择对应的原子标志变量
    std::atomic<bool> & confirmed =
        (state == DeliveryState::kWaitingLoad) ? load_confirmed_ : unload_confirmed_;

    // 重置确认标志，防止上一次配送的残留信号被误读
    // memory_order_release 确保重置操作在后续读取之前完成
    confirmed.store(false, std::memory_order_release);

    const std::string action_label =
        (state == DeliveryState::kWaitingLoad) ? "装货" : "卸货";

    // 日志中提示用户如何手动触发确认，方便调试
    RCLCPP_INFO(get_logger(),
        "订单 [%s] 在站点 [%s] 等待%s确认... (调用 ros2 service call /%s std_srvs/srv/Trigger)",
        order_id.c_str(), station_id.c_str(), action_label.c_str(),
        confirm_service.c_str());

    // 使用 steady_clock（挂钟时间）计算超时，而非 ROS 时钟
    // 原因：仿真时钟可能暂停或加速，挂钟时间更可靠
    const auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok())
    {
        // memory_order_acquire 确保读取到最新的标志值
        if (confirmed.load(std::memory_order_acquire))
        {
            RCLCPP_INFO(get_logger(), "订单 [%s] %s确认完成",
                        order_id.c_str(), action_label.c_str());
            return true;
        }

        // 检查是否超时
        const auto elapsed = std::chrono::steady_clock::now() - start;
        if (std::chrono::duration_cast<std::chrono::duration<double>>(elapsed)
                .count() > wait_confirmation_timeout_sec_)
        {
            RCLCPP_WARN(get_logger(),
                "订单 [%s] %s确认超时 (%.0f 秒)",
                order_id.c_str(), action_label.c_str(),
                wait_confirmation_timeout_sec_);
            return false;
        }

        // 500ms 轮询间隔：在响应速度和 CPU 占用之间取得平衡
        rclcpp::sleep_for(500ms);
    }
    // 节点关闭时退出循环
    return false;
}

// ======================== 服务回调实现 ========================

/**
 * @brief 订单提交服务回调。
 *
 * 多层验证确保只有合法订单才能进入队列：
 * 1. order_id 非空检查
 * 2. 站点存在性检查
 * 3. 站点类型匹配检查（取货站不能是 dropoff 类型，反之亦然）
 * 4. order_id 唯一性检查（队列和历史记录中均不能重复）
 *
 * 优先级排序采用插入排序方式，保证队列始终有序。
 */
void DeliveryManager::handle_submit_order(
    const std::shared_ptr<delivery_interfaces::srv::SubmitOrder::Request> request,
    std::shared_ptr<delivery_interfaces::srv::SubmitOrder::Response> response)
{
    const auto & order = request->order;

    // 验证 order_id 非空——空 ID 无法用于状态追踪和报告查询
    if (order.order_id.empty())
    {
        response->accepted = false;
        response->reason = "order_id 不能为空";
        return;
    }

    // 验证取货站点存在
    auto pickup_it = stations_.find(order.pickup_station);
    if (pickup_it == stations_.end())
    {
        response->accepted = false;
        response->reason = "取货站点不存在: " + order.pickup_station;
        return;
    }
    // 验证送货站点存在
    auto dropoff_it = stations_.find(order.dropoff_station);
    if (dropoff_it == stations_.end())
    {
        response->accepted = false;
        response->reason = "送货站点不存在: " + order.dropoff_station;
        return;
    }

    // 验证站点类型语义正确性：
    // type=0 是取货站，type=1 是送货站，type=2 是充电桩
    // 防止用户误将送货站指定为取货点（反之亦然）
    if (pickup_it->second.type == 1)
    {
        response->accepted = false;
        response->reason = "站点 " + order.pickup_station + " 是送货站，不能作为取货点";
        return;
    }
    if (dropoff_it->second.type == 0)
    {
        response->accepted = false;
        response->reason = "站点 " + order.dropoff_station + " 是取货站，不能作为送货点";
        return;
    }

    // 构造订单记录
    OrderRecord record;
    record.order = order;
    record.state = DeliveryState::kIdle;

    {
        std::lock_guard<std::mutex> lock(queue_mutex_);

        // 检查 order_id 在待执行队列中是否重复
        for (const auto & existing : order_queue_)
        {
            if (existing.order.order_id == order.order_id)
            {
                response->accepted = false;
                response->reason = "订单 ID 重复: " + order.order_id;
                return;
            }
        }
        // 检查 order_id 在历史记录中是否重复
        // 防止同一订单被重复提交
        for (const auto & existing : completed_orders_)
        {
            if (existing.order.order_id == order.order_id)
            {
                response->accepted = false;
                response->reason = "订单 ID 已存在于历史记录: " + order.order_id;
                return;
            }
        }

        // 按优先级插入：遍历队列找到第一个优先级低于当前订单的位置
        // priority 值越大优先级越高，高优先级排在队列前端
        auto it = order_queue_.begin();
        while (it != order_queue_.end() && it->order.priority >= order.priority)
        {
            ++it;
        }
        order_queue_.insert(it, record);
    }

    response->accepted = true;
    response->reason = "订单已加入队列";

    RCLCPP_INFO(get_logger(), "接收订单 [%s]: %s → %s (优先级: %u)",
                order.order_id.c_str(),
                order.pickup_station.c_str(),
                order.dropoff_station.c_str(),
                order.priority);
}

/**
 * @brief 订单取消服务回调。
 *
 * 仅能取消队列中尚未开始执行的订单。
 * 正在执行中的订单需要通过其他机制（如 Nav2 goal 取消）来中断，
 * 这属于 Phase 2 的功能范围。
 */
void DeliveryManager::handle_cancel_order(
    const std::shared_ptr<delivery_interfaces::srv::CancelOrder::Request> request,
    std::shared_ptr<delivery_interfaces::srv::CancelOrder::Response> response)
{
    std::lock_guard<std::mutex> lock(queue_mutex_);

    // 在队列中查找目标订单
    auto it = std::find_if(order_queue_.begin(), order_queue_.end(),
        [&request](const OrderRecord & r) {
            return r.order.order_id == request->order_id;
        });

    if (it != order_queue_.end())
    {
        order_queue_.erase(it);
        response->success = true;
        response->reason = "订单已从队列中移除";
        RCLCPP_INFO(get_logger(), "取消订单 [%s]", request->order_id.c_str());
    }
    else
    {
        // 可能的原因：订单正在执行、已完成、或从未提交过
        response->success = false;
        response->reason = "订单不在队列中（可能已在执行或已完成）";
    }
}

/**
 * @brief 配送报告查询服务回调。
 *
 * 汇总所有订单状态：包括排队中的和已完成/失败的。
 * 注意：正在执行中的订单不在 order_queue_ 也不在 completed_orders_ 中
 * （它已被取出但尚未放入完成列表），因此当前实现不包含正在执行的订单。
 * 这是 Phase 1 的已知限制，Phase 2 会通过维护 current_order 引用来解决。
 */
void DeliveryManager::handle_get_report(
    const std::shared_ptr<delivery_interfaces::srv::GetDeliveryReport::Request>,
    std::shared_ptr<delivery_interfaces::srv::GetDeliveryReport::Response> response)
{
    std::lock_guard<std::mutex> lock(queue_mutex_);

    // 排队中的订单
    for (const auto & record : order_queue_)
    {
        DeliveryStatus status;
        status.order_id = record.order.order_id;
        status.state = state_to_msg(record.state);
        status.error_msg = record.error_msg;
        response->reports.push_back(status);
    }

    // 已完成/失败的订单
    for (const auto & record : completed_orders_)
    {
        DeliveryStatus status;
        status.order_id = record.order.order_id;
        status.state = state_to_msg(record.state);
        status.error_msg = record.error_msg;
        response->reports.push_back(status);
    }
}

// ======================== 配置加载 ========================

/**
 * @brief 从 YAML 文件加载站点配置的实现。
 *
 * YAML 格式要求：
 *   stations:
 *     - station_id: "station_A"
 *       x: 1.0
 *       y: 2.0
 *       yaw: 0.0           # 可选，默认 0.0
 *       station_type: 0    # 可选，默认 0 (pickup)
 *
 * 加载过程中会进行严格校验：
 * - station_type 必须在 [0, 1, 2] 范围内
 * - station_id 不能重复
 * - 至少需要一个站点
 */
bool DeliveryManager::load_station_config(const std::string & path)
{
    if (path.empty())
    {
        RCLCPP_ERROR(get_logger(), "站点配置路径为空");
        return false;
    }

    YAML::Node root;
    try
    {
        root = YAML::LoadFile(path);
    }
    catch (const std::exception & e)
    {
        RCLCPP_ERROR(get_logger(), "YAML 解析错误: %s", e.what());
        return false;
    }

    // 检查 YAML 根节点中是否包含 "stations" 键，且其值为列表类型
    const auto stations_node = root["stations"];
    if (!stations_node || !stations_node.IsSequence())
    {
        RCLCPP_ERROR(get_logger(), "YAML 中缺少 'stations' 列表");
        return false;
    }

    // 清空已有数据，支持重新加载（虽然当前版本只在启动时加载一次）
    stations_.clear();
    for (size_t i = 0; i < stations_node.size(); ++i)
    {
        const auto & node = stations_node[i];
        Station station;
        station.id = node["station_id"].as<std::string>();
        station.pose.x = node["x"].as<double>();
        station.pose.y = node["y"].as<double>();
        // yaw 和 station_type 是可选字段，提供合理的默认值
        station.pose.yaw = node["yaw"] ? node["yaw"].as<double>() : 0.0;
        station.type = node["station_type"] ? node["station_type"].as<uint8_t>() : 0;

        // 校验 station_type 合法性 (0=pickup, 1=dropoff, 2=charge)
        if (station.type > 2)
        {
            RCLCPP_ERROR(get_logger(), "站点 [%s] 类型非法: %u (应为 0/1/2)",
                         station.id.c_str(), station.type);
            return false;
        }

        // 校验 station_id 不重复——重复 ID 会导致后续查找歧义
        if (stations_.count(station.id) > 0)
        {
            RCLCPP_ERROR(get_logger(), "站点 ID 重复: %s", station.id.c_str());
            return false;
        }

        stations_[station.id] = station;
        RCLCPP_INFO(get_logger(), "  站点 [%s]: (%.2f, %.2f, %.2f) 类型=%u",
                    station.id.c_str(), station.pose.x, station.pose.y,
                    station.pose.yaw, station.type);
    }

    // 至少需要一个站点才算加载成功
    return !stations_.empty();
}

// ======================== 环境准备（复用 Ros2Learning/task_runner 模式） ========================

/**
 * @brief 等待仿真时钟实现。
 *
 * 复用 Ros2Learning 中 task_runner 的模式：
 * Gazebo 启动后会发布 /clock 话题，ROS 节点的时钟才会从 0 变为实际仿真时间。
 * 100ms 轮询间隔足够快速响应，不会显著增加 CPU 负载。
 */
bool DeliveryManager::wait_for_time()
{
    // 使用挂钟时间（steady_clock）计算超时，因为此时 ROS 时钟尚不可用
    const auto start_wall = std::chrono::steady_clock::now();
    while (rclcpp::ok())
    {
        // 时钟值非零说明已收到 /clock 消息
        if (this->get_clock()->now().nanoseconds() != 0)
        {
            return true;
        }

        const auto elapsed = std::chrono::steady_clock::now() - start_wall;
        if (std::chrono::duration_cast<std::chrono::duration<double>>(elapsed)
                .count() > clock_wait_timeout_sec_)
        {
            return false;
        }
        rclcpp::sleep_for(100ms);
    }
    return false;
}

/**
 * @brief 等待 TF 链路就绪的实现。
 *
 * 创建局部 TF Buffer 和 Listener 来查询 map→base_link 变换。
 * 使用 tf2::TimePointZero 查询最新可用的变换（而非指定时间点），
 * 因为此时只需要确认链路是否存在，不关心变换的时间戳。
 *
 * @note TF Buffer 和 Listener 是局部变量，方法返回后自动销毁。
 *       运行期间的 TF 查询由 Nav2 内部处理，不需要本节点维护。
 */
bool DeliveryManager::wait_for_tf()
{
    tf2_ros::Buffer tf_buffer(this->get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer);

    RCLCPP_INFO(get_logger(), "等待 TF 链路: [%s] → [%s]...",
                map_frame_.c_str(), base_frame_.c_str());

    const auto start_time = this->now();
    while (rclcpp::ok())
    {
        try
        {
            // 尝试查询 map→base_link 的变换
            tf_buffer.lookupTransform(map_frame_, base_frame_,
                                      tf2::TimePointZero);
            RCLCPP_INFO(get_logger(), "TF 链路就绪");
            return true;
        }
        catch (const tf2::TransformException &)
        {
            // 变换尚不可用，等待后重试
            rclcpp::sleep_for(200ms);
        }

        // 此处使用 ROS 时钟计算超时（此时 ROS 时钟已可用）
        if ((this->now() - start_time).seconds() > tf_wait_timeout_sec_)
        {
            return false;
        }
    }
    return false;
}

/**
 * @brief 等待 Nav2 Action Server 可用的实现。
 *
 * 直接调用 rclcpp_action 客户端的 wait_for_action_server 方法，
 * 这是 ROS 2 推荐的标准做法。
 */
bool DeliveryManager::wait_for_action_server()
{
    RCLCPP_INFO(get_logger(), "等待 Nav2 action server [%s]...",
                nav2_action_name_.c_str());
    // 将 double 超时值转换为 chrono 类型
    const std::chrono::duration<double> timeout(action_server_wait_timeout_sec_);
    return action_client_->wait_for_action_server(
        std::chrono::duration_cast<std::chrono::milliseconds>(timeout));
}

/**
 * @brief 发布初始位姿的实现。
 *
 * 关键细节：
 * - 协方差矩阵只设置了对角线上的 x(索引0)、y(索引7)、yaw(索引35) 三个值
 *   其他元素保持为 0，表示各维度之间无相关性
 * - 0.25 的协方差值对应 0.5 米/弧度的标准差，这是比较宽松的初始估计
 * - 连续发布 5 次是经验做法：AMCL 启动瞬间可能丢失订阅，多次发布提高可靠性
 */
void DeliveryManager::publish_initial_pose()
{
    PoseWithCovarianceStamped msg;
    msg.header.frame_id = map_frame_;
    msg.header.stamp = this->now();
    msg.pose.pose.position.x = initial_x_;
    msg.pose.pose.position.y = initial_y_;

    // 将欧拉角（yaw）转换为四元数，roll 和 pitch 设为 0（平面运动）
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, initial_yaw_);
    msg.pose.pose.orientation = tf2::toMsg(q);

    // 设置协方差矩阵的对角线元素（6x6 矩阵，按行主序展开为 36 元素数组）
    // 索引 0: x 方向的方差, 索引 7: y 方向的方差, 索引 35: yaw 方向的方差
    msg.pose.covariance[0] = 0.25;
    msg.pose.covariance[7] = 0.25;
    msg.pose.covariance[35] = 0.25;

    RCLCPP_INFO(get_logger(), "发布初始位姿到 /initialpose");
    // 连续发布 5 次，间隔 200ms，确保 AMCL 接收到
    for (int i = 0; rclcpp::ok() && i < 5; ++i)
    {
        // 每次更新时间戳，确保消息不会因时间戳过旧被丢弃
        msg.header.stamp = this->now();
        initial_pose_pub_->publish(msg);
        rclcpp::sleep_for(200ms);
    }
}

// ======================== 导航（复用 Ros2Learning/task_runner 模式）========================

/**
 * @brief 构造 PoseStamped 消息的实现。
 *
 * 将简化的 Pose2D 转换为完整的 ROS PoseStamped：
 * - 添加 frame_id（map 坐标系）和当前时间戳
 * - 将 yaw 欧拉角转换为四元数（Nav2 要求四元数表示朝向）
 */
DeliveryManager::PoseStamped DeliveryManager::make_pose(const Pose2D & pose) const
{
    PoseStamped msg;
    msg.header.frame_id = map_frame_;
    msg.header.stamp = this->now();
    msg.pose.position.x = pose.x;
    msg.pose.position.y = pose.y;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, pose.yaw);
    msg.pose.orientation = tf2::toMsg(q);

    return msg;
}

/**
 * @brief 导航到指定位姿的实现。
 *
 * 完整的 Action 调用流程：
 * 1. 构造目标并发送（async_send_goal）
 * 2. 等待目标被接受（5 秒超时）
 * 3. 等待导航完成（navigation_timeout_sec_ 超时）
 * 4. 超时则发送取消指令
 *
 * feedback_callback 使用 RCLCPP_INFO_THROTTLE 限制日志频率为 2 秒一次，
 * 避免导航过程中日志刷屏。
 */
bool DeliveryManager::navigate_to(const Pose2D & pose, const std::string & label)
{
    // 构造导航目标
    NavigateToPose::Goal goal_msg;
    goal_msg.pose = make_pose(pose);

    // 配置 Action 发送选项
    auto send_goal_options =
        rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    // 设置导航过程中的反馈回调：每 2 秒打印一次剩余距离
    send_goal_options.feedback_callback =
        [this, label](GoalHandle::SharedPtr,
                      const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
        // RCLCPP_INFO_THROTTLE 在指定时间窗口内只打印一次，避免日志刷屏
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                             "[%s] 剩余距离: %.2f 米", label.c_str(),
                             feedback->distance_remaining);
    };

    // 异步发送导航目标
    auto goal_future =
        action_client_->async_send_goal(goal_msg, send_goal_options);

    // 等待目标被 Action Server 接受
    // executor 在后台线程 spin，会自动处理 goal response，无需手动 spin
    if (goal_future.wait_for(5s) != std::future_status::ready)
    {
        RCLCPP_ERROR(get_logger(), "[%s] 目标发送超时", label.c_str());
        return false;
    }

    auto goal_handle = goal_future.get();
    if (!goal_handle)
    {
        // Action Server 拒绝了目标（可能是目标点不可达或参数无效）
        RCLCPP_ERROR(get_logger(), "[%s] 导航目标被拒绝", label.c_str());
        return false;
    }

    // 异步等待导航结果
    auto result_future = action_client_->async_get_result(goal_handle);
    const std::chrono::duration<double> timeout(navigation_timeout_sec_);

    if (result_future.wait_for(timeout) == std::future_status::ready)
    {
        const auto wrapped_result = result_future.get();
        // 只有 SUCCEEDED 才算导航成功，ABORTED/CANCELED 都视为失败
        return (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED);
    }

    // 导航超时：主动发送取消指令，避免 Nav2 继续执行过时的目标
    RCLCPP_WARN(get_logger(), "[%s] 导航超时，发送取消指令...", label.c_str());
    auto cancel_future = action_client_->async_cancel_goal(goal_handle);
    cancel_future.wait_for(5s);  // 等待取消响应，但不关心结果
    return false;
}

// ======================== 状态管理 ========================

/**
 * @brief 发布配送状态的实现。
 *
 * 每次状态变更时调用，构造 DeliveryStatus 消息并发布。
 * 同时打印日志，便于调试时在终端直接观察状态变化。
 */
void DeliveryManager::publish_status(
    const std::string & order_id, DeliveryState state,
    const std::string & station, float progress, const std::string & error)
{
    DeliveryStatus msg;
    msg.stamp = this->now();
    msg.order_id = order_id;
    msg.state = state_to_msg(state);     // 枚举转为消息常量
    msg.current_station = station;
    msg.progress = progress;
    msg.error_msg = error;
    status_pub_->publish(msg);

    // 日志输出：进度以百分比显示，更直观
    RCLCPP_INFO(get_logger(), "状态更新 [%s]: %s (站点: %s, 进度: %.0f%%)",
                order_id.c_str(), state_to_string(state).c_str(),
                station.c_str(), progress * 100.0f);
}

/**
 * @brief 内部状态枚举到消息常量的映射。
 *
 * DeliveryStatus.msg 中通过 uint8 常量定义了所有可能的状态值，
 * 本方法确保内部枚举与消息定义保持一致。
 * default 分支返回 STATE_IDLE 作为安全兜底。
 */
uint8_t DeliveryManager::state_to_msg(DeliveryState state) const
{
    switch (state)
    {
    case DeliveryState::kIdle:           return DeliveryStatus::STATE_IDLE;
    case DeliveryState::kGoingToPickup:  return DeliveryStatus::STATE_GOING_TO_PICKUP;
    case DeliveryState::kWaitingLoad:    return DeliveryStatus::STATE_WAITING_LOAD;
    case DeliveryState::kGoingToDropoff: return DeliveryStatus::STATE_GOING_TO_DROPOFF;
    case DeliveryState::kWaitingUnload:  return DeliveryStatus::STATE_WAITING_UNLOAD;
    case DeliveryState::kComplete:       return DeliveryStatus::STATE_COMPLETE;
    case DeliveryState::kFailed:         return DeliveryStatus::STATE_FAILED;
    default:                             return DeliveryStatus::STATE_IDLE;
    }
}

/**
 * @brief 状态枚举到可读字符串的映射，用于日志输出。
 *
 * 格式为"中文(English)"，方便中英文环境下都能快速识别状态。
 */
std::string DeliveryManager::state_to_string(DeliveryState state) const
{
    switch (state)
    {
    case DeliveryState::kIdle:           return "空闲(Idle)";
    case DeliveryState::kGoingToPickup:  return "前往取货点(GoingToPickup)";
    case DeliveryState::kWaitingLoad:    return "等待装货(WaitingLoad)";
    case DeliveryState::kGoingToDropoff: return "前往送货点(GoingToDropoff)";
    case DeliveryState::kWaitingUnload:  return "等待卸货(WaitingUnload)";
    case DeliveryState::kComplete:       return "配送完成(Complete)";
    case DeliveryState::kFailed:         return "配送失败(Failed)";
    default:                             return "未知(Unknown)";
    }
}

}  // namespace delivery_core

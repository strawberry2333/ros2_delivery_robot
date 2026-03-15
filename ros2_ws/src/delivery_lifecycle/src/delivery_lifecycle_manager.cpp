/**
 * @file delivery_lifecycle_manager.cpp
 * @brief 配送系统生命周期管理器实现。
 *
 * 复用 Ros2Learning/lifecycle_manager_node 模式，
 * 适配配送系统的节点启动编排需求。
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

DeliveryLifecycleManager::DeliveryLifecycleManager(const rclcpp::NodeOptions & options)
    : rclcpp::Node("delivery_lifecycle_manager", options)
{
    std::vector<std::string> node_names;
    load_managed_node_names(node_names);
    setup_service_executor();
    initialize_managed_nodes(node_names);
    start_background_threads();

    RCLCPP_INFO(get_logger(),
        "[DeliveryLifecycleManager] 将管理 %zu 个节点", managed_nodes_.size());
}

DeliveryLifecycleManager::~DeliveryLifecycleManager()
{
    if (startup_thread_.joinable())
    {
        startup_thread_.join();
    }
    service_executor_.cancel();
    if (service_executor_thread_.joinable())
    {
        service_executor_thread_.join();
    }
}

void DeliveryLifecycleManager::startup_sequence()
{
    if (!wait_for_all_services(10s))
    {
        return;
    }
    if (!configure_all_nodes())
    {
        return;
    }
    wait_transition_delay();
    if (!activate_all_nodes())
    {
        return;
    }

    RCLCPP_INFO(get_logger(),
        "[DeliveryLifecycleManager] 所有节点已 Active");
}

void DeliveryLifecycleManager::load_managed_node_names(
    std::vector<std::string> & node_names)
{
    this->declare_parameter<std::vector<std::string>>(
        "managed_nodes", std::vector<std::string>{});
    this->declare_parameter("transition_delay_sec", 2.0);
    node_names = this->get_parameter("managed_nodes").as_string_array();
}

void DeliveryLifecycleManager::initialize_managed_nodes(
    const std::vector<std::string> & node_names)
{
    for (const auto & name : node_names)
    {
        ManagedNode node;
        node.name = name;
        node.change_state_client = this->create_client<ChangeState>(
            "/" + name + "/change_state",
            rclcpp::ServicesQoS(), service_cb_group_);
        node.get_state_client = this->create_client<GetState>(
            "/" + name + "/get_state",
            rclcpp::ServicesQoS(), service_cb_group_);
        managed_nodes_.push_back(std::move(node));
    }
}

void DeliveryLifecycleManager::setup_service_executor()
{
    service_cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    service_executor_.add_callback_group(
        service_cb_group_, this->get_node_base_interface());
}

void DeliveryLifecycleManager::start_background_threads()
{
    service_executor_thread_ = std::thread([this]() {
        service_executor_.spin();
    });

    startup_thread_ = std::thread([this]() {
        std::this_thread::sleep_for(2s);
        startup_sequence();
    });
}

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

void DeliveryLifecycleManager::wait_transition_delay()
{
    const double delay_sec =
        this->get_parameter("transition_delay_sec").as_double();
    RCLCPP_INFO(get_logger(),
        "[DeliveryLifecycleManager] 等待 %.1f 秒后 activate...", delay_sec);
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int64_t>(delay_sec * 1000.0)));
}

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

bool DeliveryLifecycleManager::configure_node_if_needed(const ManagedNode & node)
{
    const uint8_t current_state = get_state(node);
    RCLCPP_INFO(get_logger(),
        "[DeliveryLifecycleManager] '%s' 状态=%u，发送 configure...",
        node.name.c_str(), current_state);

    if (current_state != State::PRIMARY_STATE_UNCONFIGURED)
    {
        RCLCPP_WARN(get_logger(),
            "[DeliveryLifecycleManager] '%s' 不在 Unconfigured，跳过",
            node.name.c_str());
        return true;
    }

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

bool DeliveryLifecycleManager::activate_node_if_needed(const ManagedNode & node)
{
    const uint8_t current_state = get_state(node);
    RCLCPP_INFO(get_logger(),
        "[DeliveryLifecycleManager] '%s' 状态=%u，发送 activate...",
        node.name.c_str(), current_state);

    if (current_state != State::PRIMARY_STATE_INACTIVE)
    {
        RCLCPP_WARN(get_logger(),
            "[DeliveryLifecycleManager] '%s' 不在 Inactive，跳过",
            node.name.c_str());
        return true;
    }

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

bool DeliveryLifecycleManager::wait_for_service_ready(
    const ManagedNode & node, std::chrono::seconds timeout)
{
    if (!node.change_state_client->wait_for_service(timeout))
    {
        return false;
    }
    if (!node.get_state_client->wait_for_service(timeout))
    {
        return false;
    }
    return true;
}

bool DeliveryLifecycleManager::change_state(
    const ManagedNode & node, uint8_t transition_id)
{
    auto request = std::make_shared<ChangeState::Request>();
    request->transition.id = transition_id;

    auto future = node.change_state_client->async_send_request(request);
    if (future.wait_for(5s) != std::future_status::ready)
    {
        RCLCPP_ERROR(get_logger(),
            "[DeliveryLifecycleManager] change_state(%u) 超时: %s",
            transition_id, node.name.c_str());
        return false;
    }
    return future.get()->success;
}

uint8_t DeliveryLifecycleManager::get_state(const ManagedNode & node)
{
    auto request = std::make_shared<GetState::Request>();
    auto future = node.get_state_client->async_send_request(request);
    if (future.wait_for(5s) != std::future_status::ready)
    {
        RCLCPP_ERROR(get_logger(),
            "[DeliveryLifecycleManager] get_state 超时: %s", node.name.c_str());
        return State::PRIMARY_STATE_UNKNOWN;
    }
    return future.get()->current_state.id;
}

}  // namespace delivery_lifecycle

RCLCPP_COMPONENTS_REGISTER_NODE(delivery_lifecycle::DeliveryLifecycleManager)

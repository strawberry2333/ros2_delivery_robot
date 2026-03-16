/**
 * @file delivery_executor_main.cpp
 * @brief 配送执行器生命周期节点入口。
 *
 * 使用 MultiThreadedExecutor 支持 Action Server 回调、
 * 确认服务回调与 BT tick 线程并发执行。
 *
 * 注意：作为 LifecycleNode，节点启动后处于 Unconfigured 状态，
 * 需要通过 lifecycle_manager 或手动 ros2 lifecycle 命令来配置和激活。
 */

#include "delivery_core/delivery_executor.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<delivery_core::DeliveryExecutor>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();

    rclcpp::shutdown();
    return 0;
}

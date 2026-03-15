/**
 * @file delivery_executor_main.cpp
 * @brief 配送执行器节点入口。
 *
 * 使用 MultiThreadedExecutor 支持 Action Server 回调、
 * 确认服务回调与 BT tick 线程并发执行。
 */

#include "delivery_core/delivery_executor.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<delivery_core::DeliveryExecutor>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}

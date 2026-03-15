/**
 * @file delivery_manager_main.cpp
 * @brief 配送管理节点入口。
 */

#include "delivery_core/delivery_manager.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<delivery_core::DeliveryManager>();
    node->run();
    rclcpp::shutdown();
    return 0;
}

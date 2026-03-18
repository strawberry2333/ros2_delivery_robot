/**
 * @file delivery_manager_main.cpp
 * @brief 配送管理节点入口。
 *
 * 使用 MultiThreadedExecutor 确保服务回调、时钟订阅、TF 监听
 * 与配送主循环可以并发处理。
 */

#include "delivery_core/delivery_manager.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<delivery_core::DeliveryManager>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

    // 在后台线程 spin executor，处理所有回调
  auto spin_thread = std::thread([&executor]() {
        executor.spin();
    });

    // 主线程执行配送循环（阻塞）
  node->run();

    // run() 返回后停止 executor
  executor.cancel();
  spin_thread.join();

  rclcpp::shutdown();
  return 0;
}

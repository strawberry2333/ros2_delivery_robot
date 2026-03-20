/**
 * @file delivery_manager_main.cpp
 * @brief 配送管理节点入口。
 *
 * 这个入口文件只负责把 ROS 节点放进执行器并启动。
 * 真正的业务逻辑都在 delivery_manager.cpp 中。
 *
 * 使用 MultiThreadedExecutor 是为了让服务回调、时钟订阅、TF 监听
 * 与配送主循环可以并发处理：
 * - 主线程运行 DeliveryManager::run()，负责订单调度与阻塞等待
 * - 后台线程 spin executor，负责处理服务回调和 ROS 通信事件
 */

#include "delivery_core/delivery_manager.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // manager 节点本身包含服务端、Action Client 和多个共享状态，
  // 因此需要一个独立的 ROS 节点实例来承载这些通信接口。
  auto node = std::make_shared<delivery_core::DeliveryManager>();

  // 使用多线程执行器，让回调可以在后台线程被调度，
  // 避免 run() 的阻塞等待把所有 ROS 通信一起卡死。
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

    // 在后台线程 spin executor，处理所有回调。
  auto spin_thread = std::thread([&executor]() {
        executor.spin();
    });

    // 主线程执行配送循环（阻塞）。
    // run() 内部会先完成启动检查，再进入订单轮询和执行流程。
  node->run();

    // run() 返回后说明节点准备退出，此时停止 executor 并等待后台线程收尾。
  executor.cancel();
  spin_thread.join();

  rclcpp::shutdown();
  return 0;
}

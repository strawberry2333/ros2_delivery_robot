/**
 * @file delivery_executor_main.cpp
 * @brief 配送执行器生命周期节点入口。
 *
 * 入口本身很薄，只负责把 DeliveryExecutor 挂到 ROS 2 executor 上。
 * 复杂逻辑都在 LifecycleNode 和 BT 执行线程里。
 *
 * 这里使用 MultiThreadedExecutor 的原因是：
 * - action server 回调需要并发处理
 * - 装货/卸货确认服务需要独立响应
 * - lifecycle 回调与 BT 执行线程不能互相卡死
 *
 * 注意：作为 LifecycleNode，节点启动后处于 Unconfigured 状态。
 * 只有经过 configure / activate 后，才会真正开始对外提供配送能力。
 */

#include "delivery_core/delivery_executor.hpp"

int main(int argc, char ** argv)
{
  // 初始化 ROS 2 运行时。
  rclcpp::init(argc, argv);

  // 先构造生命周期节点实例，再交给 executor 管理其回调。
  auto node = std::make_shared<delivery_core::DeliveryExecutor>();

  // 多线程执行器用于同时处理 action、service 和 lifecycle 回调。
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  // executor 退出后再统一关闭 ROS 运行时。
  rclcpp::shutdown();
  return 0;
}

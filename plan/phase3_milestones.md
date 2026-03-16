# Phase 3 里程碑详细记录

## M1: BT 错误恢复（RetryNode 重试机制）

**状态**: ✅ 完成

**改动**:
- 新建 `single_delivery_robust.xml`，用 `RetryNode(num_attempts=2)` 包裹 NavigateToStation
- `delivery.launch.py` 添加 `tree_file` launch arg，默认指向 robust 版本
- 保留旧 `single_delivery.xml` 不动

## M2: CancelOrder 增强（取消执行中订单）

**状态**: ✅ 完成

**改动**:
- `DeliveryStatus.msg` 添加 `STATE_CANCELED=7`
- `delivery_manager.hpp` 添加 `kCanceled` 枚举、`current_order_id_` + `current_order_` + 关联 mutex
- `delivery_manager.cpp` 中 `handle_cancel_order()` 验证 order_id 匹配后再取消
- `execute_delivery()` 处理 `ResultCode::CANCELED` 返回值
- `state_to_msg()` / `state_to_string()` 添加 kCanceled 映射

## M3: 多订单连续处理 + 报告增强

**状态**: ✅ 完成

**改动**:
- `delivery_manager.cpp` 的 `run()` 中设置/清除 `current_order_`
- `handle_get_report()` 包含当前执行中的订单
- `handle_submit_order()` 检查 current_order_id_ 避免重复
- `delivery_executor.cpp` 在 BT 执行前重置 load/unload confirmed 标志

## M4: CheckBattery 条件节点 + 充电 BT

**状态**: ✅ 完成

**改动**:
- 新建 `check_battery.hpp` / `check_battery.cpp` — BT ConditionNode
- 新建 `delivery_mission.xml` — 电量检查 + SubTree 引用
- `delivery_executor` 注册 CheckBattery、初始化 battery_level、配送后扣减电量
- `CMakeLists.txt` 添加 check_battery.cpp

## M5: delivery_executor 转 LifecycleNode

**状态**: ✅ 完成

**改动**:
- `delivery_executor.hpp` 基类改为 `rclcpp_lifecycle::LifecycleNode`
- 添加 on_configure/on_activate/on_deactivate/on_cleanup/on_shutdown
- 创建辅助 `bt_node_`（rclcpp::Node）供 BT 叶节点使用
- `delivery_executor_main.cpp` 使用 `get_node_base_interface()`
- `CMakeLists.txt` + `package.xml` 添加 rclcpp_lifecycle
- `delivery.launch.py` 添加 lifecycle_manager 节点
- 新建 `lifecycle_manager.yaml` 配置

## M6: 自定义 Gazebo 仓库场景 + RViz 配置

**状态**: ✅ 完成

**改动**:
- 新建 `warehouse.sdf` — 10m×8m 围墙 + 货架 + 彩色站点标记
- 新建 `warehouse.yaml` + `warehouse.pgm` — Nav2 地图
- 新建 `delivery.rviz` — RViz 可视化配置
- 更新 `stations.yaml` 坐标匹配仓库场景
- `simulation.launch.py` 保持 TurtleBot3 标准世界（仓库场景可选）
- `navigation.launch.py` 保持 map 参数支持

## M7: 单元测试 + README 文档

**状态**: ✅ 完成

**改动**:
- 新建 `test_check_battery.cpp` — 4 个测试用例
- 新建 `test_report_delivery_status.cpp` — 2 个测试用例
- `CMakeLists.txt` 添加 ament_cmake_gtest 测试目标
- `package.xml` 添加测试依赖
- 更新 `README.md` — 完整架构图、运行指南、设计决策

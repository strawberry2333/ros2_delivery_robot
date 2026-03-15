# Phase 3：多订单 + 生命周期 + 工程化

**状态：⬜ 未开始**

## 目标

完整系统，支持多订单队列、错误恢复、生命周期管理、定制仿真场景、测试。

## 待办清单

### 1. 多订单队列

- [ ] delivery_manager 支持连续提交多个订单 (SubmitOrder 多次调用)
- [ ] 按优先级排序执行（Phase 1 已实现基础排序）
- [ ] 当前订单完成后自动取下一个
- [ ] 创建 `delivery_mission.xml` 行为树：
  ```xml
  <Repeat num_cycles="-1">
    <Sequence>
      <GetNextOrder order_id="{order_id}" pickup="{pickup}" dropoff="{dropoff}" />
      <SubTree ID="SingleDelivery" order_id="{order_id}" ... />
    </Sequence>
  </Repeat>
  ```

### 2. 错误恢复 (Fallback)

- [ ] BT 中加入 Fallback 节点：
  ```xml
  <Fallback>
    <NavigateToStation station_id="{station}" />
    <Sequence name="retry">
      <Wait duration_sec="3" />
      <NavigateToStation station_id="{station}" />
    </Sequence>
  </Fallback>
  ```
- [ ] 导航失败 → 重试一次 → 仍失败则标记订单为 FAILED 并跳到下一个
- [ ] 失败订单记录到 completed_orders_ 中，可通过 GetDeliveryReport 查询

### 3. CancelOrder 完善

- [ ] 支持取消正在执行的订单（不仅限于队列中的）
- [ ] 通过 Action cancel 终止导航
- [ ] 机器人停止当前动作，开始下一个订单

### 4. CheckBattery 条件节点

- [ ] 创建 `CheckBattery` BT 条件节点
- [ ] 模拟电量：每次导航消耗一定电量（参数化）
- [ ] 低于阈值时返回 FAILURE，触发返回充电点 (charge_home)
- [ ] 充电完成后继续队列中的下一个订单

### 5. delivery_lifecycle 集成

- [ ] 配置 managed_nodes 参数：
  ```yaml
  delivery_lifecycle_manager:
    ros__parameters:
      managed_nodes: ["delivery_executor"]
      transition_delay_sec: 2.0
  ```
- [ ] delivery_executor 改为 LifecycleNode
- [ ] 管理启动顺序：Nav2 就绪 → delivery_executor activate → delivery_manager 开始接单
- [ ] 健康检查：定期查询 managed nodes 状态
- [ ] 优雅关闭：deactivate → cleanup

### 6. 定制 Gazebo 仓库场景

- [ ] 创建简单的室内环境 world 文件 (`delivery_simulation/worlds/warehouse.sdf`)
- [ ] 包含：墙壁围合、4-5 个标记站点区域、简单地面纹理
- [ ] 生成对应地图文件 (`delivery_simulation/maps/warehouse.yaml` + `.pgm`)
- [ ] 在站点位置放置可视化标记（不同颜色的方块代表 pickup/dropoff/charge）
- [ ] 更新 stations.yaml 坐标匹配新地图

### 7. RViz 配置

- [ ] 创建 `delivery_simulation/rviz/delivery.rviz`
- [ ] 显示：机器人模型、地图、全局/局部路径、站点标记、delivery_status 文字

### 8. 测试

- [ ] **BT 节点单元测试** — 使用 ament_cmake_gtest
  - `test_navigate_to_station.cpp` — 模拟 Action Server 验证节点行为
  - `test_wait_for_confirmation.cpp` — 验证超时和确认逻辑
  - `test_report_delivery_status.cpp` — 验证消息发布

- [ ] **delivery_manager 集成测试**
  - 提交多个订单 → 验证执行顺序
  - 取消订单 → 验证队列变化
  - 模拟导航失败 → 验证错误处理

### 9. README

- [ ] 问题描述：什么场景、解决什么问题
- [ ] 架构图 (mermaid)：模块关系、消息流、状态机
- [ ] 环境要求与安装步骤
- [ ] 运行指南（分步骤 + 一键启动）
- [ ] 演示 GIF / 视频链接
- [ ] 设计决策说明（为什么这样选型）

## 关键文件变更

```
delivery_core/
  include/delivery_core/
    nodes/
      check_battery.hpp          # 新增
      get_next_order.hpp         # 新增
  src/
    nodes/
      check_battery.cpp          # 新增
      get_next_order.cpp         # 新增
    delivery_executor.cpp        # 修改：LifecycleNode
  behavior_trees/
    delivery_mission.xml         # 新增
  test/
    test_navigate_to_station.cpp # 新增
    test_wait_for_confirmation.cpp # 新增

delivery_simulation/
  worlds/
    warehouse.sdf                # 新增
  maps/
    warehouse.yaml               # 新增
    warehouse.pgm                # 新增
  rviz/
    delivery.rviz                # 新增

delivery_bringup/
  config/
    stations.yaml                # 修改：匹配新地图坐标
    nav2_params.yaml             # 新增：适配仿真环境
  launch/
    simulation.launch.py         # 修改：使用定制 world
    delivery.launch.py           # 修改：添加 lifecycle manager

README.md                       # 新增
```

## 验证方式

```bash
# 连续提交 3 个订单
ros2 service call /submit_order delivery_interfaces/srv/SubmitOrder \
  "{order: {order_id: 'order_001', pickup_station: 'station_A', dropoff_station: 'station_C'}}"
ros2 service call /submit_order delivery_interfaces/srv/SubmitOrder \
  "{order: {order_id: 'order_002', pickup_station: 'station_B', dropoff_station: 'station_D'}}"
ros2 service call /submit_order delivery_interfaces/srv/SubmitOrder \
  "{order: {order_id: 'order_003', pickup_station: 'station_A', dropoff_station: 'station_D', priority: 1}}"

# → order_003 (高优先级) 应先执行
# → 依次完成 3 个订单

# 获取完整报告
ros2 service call /get_delivery_report delivery_interfaces/srv/GetDeliveryReport
# → 返回 3 条配送记录

# 模拟导航失败 → 验证重试
# (在 Gazebo 中放置障碍物阻挡路径)
# → 机器人重试一次 → 仍失败则标记 FAILED → 继续下一个订单

# 运行测试
cd ros2_ws && colcon test --packages-select delivery_core
colcon test-result --verbose
```

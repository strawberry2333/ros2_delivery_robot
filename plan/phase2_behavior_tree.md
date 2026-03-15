# Phase 2：行为树 + 停靠确认

**状态：⬜ 未开始**

## 目标

到站后执行 BT 子树——低速靠近、发布状态、等待确认。delivery_manager 通过 Action 调用 delivery_executor。

## 待办清单

### 1. BT 叶节点实现

基于 `move_base_node.cpp` (StatefulActionNode) 模式，在 `delivery_core/src/nodes/` 下创建：

- [ ] **NavigateToStation** — 封装 Nav2 NavigateToPose
  - 输入端口：`station_id` (string)
  - 从黑板读取站点坐标（由 delivery_executor 预先写入）
  - 复用 MoveBase 节点的异步 Action Client 模式
  - 参考：`ros2_learning_behavior_tree/src/nodes/move_base_node.cpp`

- [ ] **DockAtStation** — 到达后低速靠近
  - 发布 cmd_vel 低速前进 0.5 秒（模拟精确停靠）
  - 发布"已到达"状态到 `/delivery_status`
  - SyncActionNode 即可（快速完成）

- [ ] **WaitForConfirmation** — 等待外部确认
  - StatefulActionNode，返回 RUNNING 直到确认
  - 监听确认标志（atomic bool），由外部 service call 触发
  - 带超时自动继续
  - 参考：`blocking_node.cpp` 的阻塞等待模式

- [ ] **ReportDeliveryStatus** — 发布配送状态
  - SyncActionNode，发布 DeliveryStatus 消息
  - 输入端口：order_id, state, station_id, progress

### 2. 行为树 XML

- [ ] **single_delivery.xml** — 单次配送行为树：
  ```xml
  <Sequence>
    <!-- 取货阶段 -->
    <NavigateToStation station_id="{pickup_station}" />
    <DockAtStation station_id="{pickup_station}" />
    <ReportDeliveryStatus order_id="{order_id}" state="WAITING_LOAD" station_id="{pickup_station}" />
    <WaitForConfirmation confirm_type="load" timeout_sec="60" />
    <!-- 送货阶段 -->
    <NavigateToStation station_id="{dropoff_station}" />
    <DockAtStation station_id="{dropoff_station}" />
    <ReportDeliveryStatus order_id="{order_id}" state="WAITING_UNLOAD" station_id="{dropoff_station}" />
    <WaitForConfirmation confirm_type="unload" timeout_sec="60" />
    <!-- 完成 -->
    <ReportDeliveryStatus order_id="{order_id}" state="COMPLETE" station_id="{dropoff_station}" />
  </Sequence>
  ```

### 3. delivery_executor 节点

- [ ] 创建 `delivery_executor.cpp` — BT 宿主节点
  - 类似 `bt_main.cpp`，但作为 Action Server 运行
  - 接收 ExecuteDelivery.action Goal
  - 将订单信息写入 BT 黑板 (order_id, pickup_station, dropoff_station)
  - 将站点坐标写入黑板 (从站点配置加载)
  - Tick 循环执行 single_delivery.xml
  - 发布 Feedback (state, progress)
  - 支持 Cancel

### 4. ExecuteDelivery.action 集成

- [ ] delivery_manager 改为通过 Action Client 调用 delivery_executor
  - 当前 Phase 1 的 `execute_delivery()` 方法改为发送 Action Goal
  - 监听 Feedback 更新 delivery_status
  - 支持取消正在执行的配送

### 5. SubmitOrder 服务

- [ ] 已在 Phase 1 实现基础版，Phase 2 确保与 Action 模式正确集成

### 6. CMakeLists.txt 更新

- [ ] delivery_core 添加 `behaviortree_cpp` 依赖
- [ ] 添加 BT 节点源文件到编译目标
- [ ] 添加 delivery_executor 可执行目标
- [ ] 安装 behavior_trees/ XML 目录

## 关键文件变更

```
delivery_core/
  include/delivery_core/
    nodes/
      navigate_to_station.hpp    # 新增
      dock_at_station.hpp        # 新增
      wait_for_confirmation.hpp  # 新增
      report_delivery_status.hpp # 新增
  src/
    nodes/
      navigate_to_station.cpp    # 新增
      dock_at_station.cpp        # 新增
      wait_for_confirmation.cpp  # 新增
      report_delivery_status.cpp # 新增
    delivery_executor.cpp        # 新增
    delivery_executor_main.cpp   # 新增
    delivery_manager.cpp         # 修改：Action Client 调用
  behavior_trees/
    single_delivery.xml          # 新增
  CMakeLists.txt                 # 修改：添加新目标
  package.xml                    # 修改：添加 behaviortree_cpp 依赖

delivery_bringup/
  launch/
    delivery.launch.py           # 修改：添加 delivery_executor 节点
```

## 验证方式

```bash
# 启动后提交订单
ros2 service call /submit_order delivery_interfaces/srv/SubmitOrder \
  "{order: {order_id: 'order_001', pickup_station: 'station_A', dropoff_station: 'station_C'}}"

# 观察 BT 驱动的状态变化
ros2 topic echo /delivery_status
# → STATE_GOING_TO_PICKUP → STATE_WAITING_LOAD

# 到站后确认
ros2 service call /confirm_load std_srvs/srv/Trigger
# → STATE_GOING_TO_DROPOFF → STATE_WAITING_UNLOAD

ros2 service call /confirm_unload std_srvs/srv/Trigger
# → STATE_COMPLETE
```

# Phase 1：骨架 + 单点导航

**状态：✅ 已完成**

## 目标

机器人在 Gazebo 中导航至指定站点并停靠，支持通过服务提交订单。

## 已完成内容

### 1. 仓库初始化 + 5 个包骨架

所有包已创建完整的 `CMakeLists.txt` + `package.xml`：

- `delivery_interfaces` — rosidl 消息生成包
- `delivery_core` — C++ 可执行节点包
- `delivery_lifecycle` — rclcpp_components 插件包
- `delivery_simulation` — 纯资源包 (ament_cmake)
- `delivery_bringup` — 纯资源包 (launch + config)

### 2. delivery_interfaces 定义

```
msg/
  DeliveryOrder.msg      — order_id, pickup_station, dropoff_station, priority
  DeliveryStatus.msg     — order_id, state(枚举常量), current_station, progress, error_msg
  StationInfo.msg        — station_id, x, y, yaw, station_type(pickup/dropoff/charge)
srv/
  SubmitOrder.srv        — Request: DeliveryOrder; Response: accepted, reason
  CancelOrder.srv        — Request: order_id; Response: success, reason
  GetDeliveryReport.srv  — Response: DeliveryStatus[]
action/
  ExecuteDelivery.action — Goal: DeliveryOrder; Feedback: state, progress; Result: success, elapsed_time
```

### 3. stations.yaml 站点配置

5 个站点：station_A (pickup), station_B (pickup), station_C (dropoff), station_D (dropoff), charge_home (charge)。坐标基于 TurtleBot3 World 地图。

### 4. delivery_manager 节点

从 `task_runner.cpp` 重构而来，核心改动：

- **状态机**: `kIdle → kGoingToPickup → kWaitingLoad → kGoingToDropoff → kWaitingUnload → kComplete`
- **去掉机械臂依赖**: 不再调用 pick/place 服务，改为等待 `/confirm_load` 和 `/confirm_unload` 服务确认
- **站点配置加载**: 从 YAML 读取站点 ID + 坐标 + 类型
- **订单队列**: 支持 SubmitOrder 服务提交，按优先级排序
- **服务端**: submit_order, cancel_order, get_delivery_report, confirm_load, confirm_unload
- **状态发布**: `/delivery_status` 话题实时发布 DeliveryStatus 消息

复用的模式（几乎不改）：
- `wait_for_time()` — 仿真时钟等待
- `wait_for_tf()` — TF 链路检查
- `wait_for_action_server()` — Nav2 服务端等待
- `publish_initial_pose()` — AMCL 初始位姿
- `navigate_to()` — 异步 Nav2 导航 + 超时取消
- `make_pose()` — Pose2D → PoseStamped 转换

### 5. delivery_lifecycle_manager

从 `lifecycle_manager_node.cpp` 复用，适配配送系统参数名。

### 6. Launch 文件

| 文件 | 职责 |
|---|---|
| `simulation.launch.py` | 启动 TurtleBot3 Gazebo World |
| `navigation.launch.py` | 启动 Nav2 全栈 |
| `delivery.launch.py` | 启动 delivery_manager 节点 |
| `demo.launch.py` | 一键启动全部（TimerAction 分层延迟） |

### 7. 验证方式

```bash
ros2 launch delivery_bringup demo.launch.py
# → 机器人在 Gazebo 中初始化

ros2 service call /submit_order delivery_interfaces/srv/SubmitOrder \
  "{order: {order_id: 'order_001', pickup_station: 'station_A', dropoff_station: 'station_C'}}"
# → 机器人导航到 station_A

ros2 topic echo /delivery_status
# → 显示 STATE_GOING_TO_PICKUP → STATE_WAITING_LOAD

ros2 service call /confirm_load std_srvs/srv/Trigger
# → 机器人继续导航到 station_C

ros2 service call /confirm_unload std_srvs/srv/Trigger
# → 配送完成，状态变为 STATE_COMPLETE
```

# ROS2 室内配送机器人

基于 ROS2 Jazzy 的室内多点配送机器人系统——接收配送订单，自主导航至取货点等待装货确认，再导航至送货点等待卸货确认，支持多订单队列、优先级调度和失败重试。

## 目录

- [典型应用场景](#典型应用场景)
- [技术栈](#技术栈)
- [系统架构](#系统架构)
  - [分层总览](#分层总览)
  - [包结构](#包结构)
  - [节点与通信拓扑](#节点与通信拓扑)
  - [默认订单链路](#默认订单链路)
  - [配送状态机](#配送状态机)
  - [自定义接口](#自定义接口)
  - [行为树架构](#行为树架构)
  - [生命周期管理](#生命周期管理)
- [构建与运行](#构建与运行)
- [站点配置](#站点配置)
- [设计决策](#设计决策)
- [开发进度](#开发进度)

## 典型应用场景

- 仓库内部物料配送（工位间转运零件）
- 医院药品/样本配送（药房 → 病房）
- 办公楼文件/快递配送（前台 → 工位）

## 技术栈

| 项 | 选型 |
|---|---|
| OS | Ubuntu 24.04 |
| ROS | ROS2 Jazzy |
| 仿真 | Gazebo Harmonic |
| 机器人 | TurtleBot3 Waffle Pi |
| 导航 | Nav2 |
| 行为树 | BehaviorTree.CPP v4 |
| 构建 | colcon / ament_cmake |

## 系统架构

### 分层总览

从实现上看，这个项目不是单一节点，而是一个分层的配送业务系统：

- `delivery_interfaces` 定义协议层，统一订单、状态、站点、服务和动作接口。
- `delivery_core` 定义业务执行层，负责接单调度、单单执行、行为树叶节点。
- `delivery_lifecycle` 定义系统启动控制层，负责把 `delivery_executor` 推进到可工作状态。
- `delivery_bringup` 定义装配层，负责 launch、参数和默认行为树选择。
- `delivery_simulation` 定义仿真资源层，提供 Gazebo world、地图和场景资源。

```mermaid
flowchart TB
    User["操作员 / 上位机 / CLI"]

    subgraph Protocol["协议层: delivery_interfaces"]
        S1["SubmitOrder.srv"]
        S2["CancelOrder.srv"]
        S3["GetDeliveryReport.srv"]
        A1["ExecuteDelivery.action"]
        M1["DeliveryOrder.msg / DeliveryStatus.msg / StationInfo.msg"]
    end

    subgraph Core["业务执行层: delivery_core"]
        Manager["DeliveryManager<br/>订单调度器"]
        Executor["DeliveryExecutor<br/>单订单执行器 / BT 宿主"]
        BTN["BT Node Classes<br/>NavigateToStation / DockAtStation / WaitForConfirmation / ReportDeliveryStatus / CheckBattery"]
    end

    subgraph Lifecycle["启动控制层: delivery_lifecycle"]
        LCM["DeliveryLifecycleManager<br/>生命周期编排器"]
    end

    subgraph Bringup["装配层: delivery_bringup"]
        Launch["demo.launch.py / delivery.launch.py"]
        Params["stations.yaml / *.yaml"]
    end

    subgraph Runtime["运行时外部依赖"]
        Nav2["Nav2 / bt_navigator / navigate_to_pose"]
        AMCL["AMCL / TF / initialpose"]
        Gazebo["Gazebo / clock"]
    end

    User --> S1
    User --> S2
    User --> S3
    S1 --> Manager
    S2 --> Manager
    S3 --> Manager
    Manager --> A1
    A1 --> Executor
    Executor --> BTN
    BTN --> Nav2
    Manager --> AMCL
    Manager --> Gazebo
    LCM -. configure / activate .-> Executor
    Launch --> Manager
    Launch --> Executor
    Launch --> LCM
    Params --> Manager
    Params --> Executor
```

### 包结构

```
ros2_ws/src/
├── delivery_interfaces/    消息/服务/动作定义
├── delivery_core/          C++ 核心节点（订单管理 + 导航编排 + BT 节点）
├── delivery_lifecycle/     生命周期管理器
├── delivery_simulation/    仓库仿真场景资源（世界模型 + 地图）
└── delivery_bringup/       Launch 文件 + 配置文件
```

### 节点与通信拓扑

```mermaid
graph TD
    subgraph 外部操作
        User["操作员 / 上位机"]
    end

    subgraph delivery_core
        Manager["delivery_manager<br/>订单队列 + 调度"]
        Executor["delivery_executor<br/>BT 宿主 LifecycleNode"]
    end

    subgraph Nav2
        NavStack["Nav2 导航栈<br/>navigate_to_pose"]
    end

    subgraph delivery_lifecycle
        LCM["delivery_lifecycle_manager<br/>节点启动编排"]
    end

    StatusTopic["/delivery_status"]

    User -- "SubmitOrder / CancelOrder" --> Manager
    User -- "confirm_load / confirm_unload" --> Executor
    Manager -- "ExecuteDelivery Action" --> Executor
    Executor -- "NavigateToPose Action" --> NavStack
    Executor -- "发布状态" --> StatusTopic
    LCM -. "configure / activate" .-> Executor
```

### 默认订单链路

默认 bringup 会加载 `single_delivery_robust.xml`，也就是“带导航重试的单次配送树”，而不是带 `CheckBattery` 的完整任务树。一次订单的默认执行路径如下：

```mermaid
sequenceDiagram
    participant User as 用户 / 上位机
    participant Manager as delivery_manager
    participant Executor as delivery_executor
    participant BT as 默认 BT
    participant Nav2 as Nav2

    User->>Manager: SubmitOrder
    Manager->>Manager: 校验订单并入优先级队列
    Manager->>Executor: ExecuteDelivery goal
    Executor->>BT: 加载 single_delivery_robust.xml
    BT->>Nav2: 导航到 pickup（失败时自动重试一次）
    BT->>Executor: 等待 /confirm_load
    User->>Executor: /confirm_load
    BT->>Nav2: 导航到 dropoff（失败时自动重试一次）
    BT->>Executor: 等待 /confirm_unload
    User->>Executor: /confirm_unload
    Executor-->>Manager: result + feedback
```

更细的类级架构、时序和生命周期实现说明见 [docs/architecture.md](docs/architecture.md)。

### 配送状态机

```mermaid
stateDiagram-v2
    [*] --> Idle
    Idle --> GoingToPickup : 取到订单
    GoingToPickup --> WaitingLoad : 到达取货点
    GoingToPickup --> Failed : 导航失败（重试耗尽）
    WaitingLoad --> GoingToDropoff : confirm_load
    GoingToDropoff --> WaitingUnload : 到达送货点
    GoingToDropoff --> Failed : 导航失败
    WaitingUnload --> Complete : confirm_unload
    Complete --> Idle : 取下一个订单
    Failed --> Idle : 跳到下一个订单
    GoingToPickup --> Canceled : 用户取消
    GoingToDropoff --> Canceled : 用户取消
    Canceled --> Idle : 跳到下一个订单
```

### 自定义接口

**消息 (msg)**

| 消息 | 字段 | 说明 |
|---|---|---|
| `DeliveryOrder` | order_id, pickup_station, dropoff_station, priority | 配送订单描述 |
| `DeliveryStatus` | order_id, state, current_station, progress, error_msg | 配送实时状态（含 STATE_CANCELED） |
| `StationInfo` | station_id, x, y, yaw, station_type | 站点坐标与类型 |

**服务 (srv)**

| 服务 | 说明 |
|---|---|
| `SubmitOrder` | 提交配送订单到队列 |
| `CancelOrder` | 取消队列中或执行中的订单（验证 order_id） |
| `GetDeliveryReport` | 获取所有订单状态报告（含当前执行中） |

**动作 (action)**

| 动作 | 说明 |
|---|---|
| `ExecuteDelivery` | 执行单次配送，支持进度反馈和取消 |

### 行为树架构

BT 负责单次配送内的决策流程。当前仓库里有三种 XML 变体：

| XML 文件 | 特点 |
|---|---|
| `single_delivery.xml` | 基础版：无重试 |
| `single_delivery_robust.xml` | 默认 demo 使用；导航失败自动重试一次 |
| `delivery_mission.xml` | 可选树；在前面增加 `CheckBattery` 条件并引用 `SingleDelivery` 子树 |

默认树的大致流程是：

```text
Sequence
├── ReportDeliveryStatus → going_to_pickup
├── RetryUntilSuccessful(2) → NavigateToStation(pickup)
├── DockAtStation(pickup)
├── ReportDeliveryStatus → waiting_load
├── WaitForConfirmation(load)
├── ReportDeliveryStatus → going_to_dropoff
├── RetryUntilSuccessful(2) → NavigateToStation(dropoff)
├── DockAtStation(dropoff)
├── ReportDeliveryStatus → waiting_unload
├── WaitForConfirmation(unload)
└── ReportDeliveryStatus → complete
```

### 生命周期管理

`delivery_executor` 是 `LifecycleNode`，由 `delivery_lifecycle_manager` 负责推进到 `Active`，保证 manager 接单前执行侧已经 ready：

```mermaid
stateDiagram-v2
    [*] --> Unconfigured
    Unconfigured --> Inactive : on_configure
    Inactive --> Active : on_activate
    Active --> Inactive : on_deactivate
    Inactive --> Unconfigured : on_cleanup
    Inactive --> Finalized : on_shutdown
    Active --> Finalized : on_shutdown
```

更细的生命周期启动时序与线程模型见 [docs/architecture.md](docs/architecture.md)。

## 构建与运行

```bash
# 环境准备
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=waffle_pi

# 构建
cd ros2_ws
colcon build --symlink-install
source install/setup.bash

# 一键启动
ros2 launch delivery_bringup demo.launch.py

# 提交订单
ros2 service call /submit_order delivery_interfaces/srv/SubmitOrder \
  "{order: {order_id: 'order_001', pickup_station: 'station_A', dropoff_station: 'station_C', priority: 0}}"

# 观察状态
ros2 topic echo /delivery_status

# 人工确认
ros2 service call /confirm_load std_srvs/srv/Trigger
ros2 service call /confirm_unload std_srvs/srv/Trigger

# 配送报告
ros2 service call /get_delivery_report delivery_interfaces/srv/GetDeliveryReport
```

### 运行测试

```bash
cd ros2_ws
colcon build --symlink-install
colcon test --packages-select delivery_core
colcon test-result --verbose
```

## 站点配置

站点在 `delivery_bringup/config/stations.yaml` 中定义：

```yaml
stations:
  - station_id: "station_A"
    x: 3.0
    y: 2.0
    yaw: 0.0
    station_type: 0   # pickup
  - station_id: "station_C"
    x: -3.0
    y: -2.0
    yaw: 3.14
    station_type: 1   # dropoff
  - station_id: "charge_home"
    x: 0.0
    y: 0.0
    yaw: 0.0
    station_type: 2   # charge
```

站点类型：0 = 取货点，1 = 送货点，2 = 充电点。

## 设计决策

- **双节点架构**：manager 负责调度，executor 负责执行，通过 Action 解耦
- **BT + 状态机混合**：BT 编排单次配送决策，状态机管理订单队列
- **LifecycleNode**：executor 使用生命周期管理，确保有序启动
- **RetryNode 重试**：导航失败自动重试，无需修改业务逻辑
- **模拟电池**：每次配送扣减电量，低电量时直接中止配送（demo 级别，不恢复电量）

## 开发进度

- [x] Phase 1：骨架 + 单点导航
- [x] Phase 2：行为树 + 停靠确认
- [x] Phase 3：多订单 + 生命周期 + 工程化

> 核心功能骨架已实现。低电量时直接中止当前配送（demo 级别，不恢复电量），仿真默认使用自定义仓库场景。

详见 [plan/](plan/) 目录。

## License

Apache-2.0

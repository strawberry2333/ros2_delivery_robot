# CLAUDE.md

## Communication

与用户沟通时尽量使用中文回复。代码评审保持专业，直指问题。

Git commit message 使用中文撰写，技术术语（如 CI、std_srvs 等）可保留英文。

## Repository

- **本项目**: https://github.com/strawberry2333/ros2_delivery_robot

## Project Overview

**一句话：** 室内多点配送机器人——接收配送订单，自主导航至取货点等待装货确认，再导航至送货点等待卸货确认，支持多订单队列和失败重试。

**定位：** 场景驱动的室内配送机器人系统，复用 `Ros2Learning` 仓库的成熟 ROS2 代码模式。

## Source Repository

本项目从 `Ros2Learning` 仓库复用代码模式（仓库地址：https://github.com/James-Xue/Ros2Learning ）。关键映射：

| Ros2Learning 源码 | 本项目用途 |
|---|---|
| `task_runner.cpp` | → `delivery_manager`：状态机重构为配送场景 |
| `nav2_client.cpp` | → 导航底层：AMCL 初始化、TF、sim_time 模式复用 |
| `move_base_node.cpp` (BT节点) | → `NavigateToStation` BT 节点模板 (Phase 2) |
| `lifecycle_manager_node.cpp` | → `delivery_lifecycle_manager` |
| `custom_interfaces/` | → `delivery_interfaces` 模板 |
| composition 模式 | → 组件注册 + 零拷贝模式 (Phase 3) |
| multithreading 模式 | → MultiThreadExecutor + CallbackGroup 隔离 |
| `blocking_node.cpp` | → `WaitForConfirmation` 节点阻塞等待模式参考 (Phase 2) |

## Repository Layout

```
ros2_delivery_robot/
├── CLAUDE.md                        # 本文件
├── plan/                            # 重构计划文档
│   ├── overview.md                  # 项目总览与设计决策
│   ├── phase1_skeleton.md           # 阶段1：骨架 + 单点导航
│   ├── phase2_behavior_tree.md      # 阶段2：行为树 + 停靠确认
│   └── phase3_full_system.md        # 阶段3：多订单 + 生命周期 + 工程化
├── ros2_ws/src/
│   ├── delivery_interfaces/         # 自定义 msg/srv/action 定义
│   ├── delivery_core/               # C++ 核心（订单管理 + 导航编排 + BT 节点）
│   ├── delivery_lifecycle/          # 生命周期管理
│   ├── delivery_simulation/         # Gazebo 仿真资源
│   └── delivery_bringup/            # Launch + 配置文件
```

## Module Responsibilities

### delivery_interfaces
自定义消息/服务/动作定义：
- `DeliveryOrder.msg` — 订单 (order_id, pickup_station, dropoff_station, priority)
- `DeliveryStatus.msg` — 状态 (order_id, state, current_station, progress, error_msg)
- `StationInfo.msg` — 站点 (station_id, x, y, yaw, station_type)
- `SubmitOrder.srv` — 提交订单
- `CancelOrder.srv` — 取消订单
- `GetDeliveryReport.srv` — 获取配送报告
- `ExecuteDelivery.action` — 执行单次配送 (Phase 2)

### delivery_core
- **delivery_manager** — 订单队列管理 + 调度（通过 ExecuteDelivery Action 委托执行）
- **delivery_executor** — BT 宿主节点，持有 Action Server + 确认服务
- **BT 叶节点** — NavigateToStation, DockAtStation, WaitForConfirmation, ReportDeliveryStatus, CheckBattery（电量不足直接 FAILURE 中止配送）

### delivery_lifecycle
- 生命周期管理器，管理节点启动顺序

### delivery_simulation
- Gazebo 场景、地图、RViz 配置

### delivery_bringup
- Launch 分层：simulation → navigation → delivery → demo 一键启动
- 配置文件：stations.yaml, delivery_manager.yaml

## Build Commands

```bash
# 构建全部
cd ros2_ws && colcon build --symlink-install

# 构建单个包（delivery_interfaces 必须先构建）
cd ros2_ws && colcon build --packages-select delivery_interfaces --symlink-install
cd ros2_ws && colcon build --packages-select delivery_core --symlink-install

# Source 环境
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
```

## Run & Verify

```bash
# 一键启动
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch delivery_bringup demo.launch.py

# 提交订单
ros2 service call /submit_order delivery_interfaces/srv/SubmitOrder \
  "{order: {order_id: 'order_001', pickup_station: 'station_A', dropoff_station: 'station_C', priority: 0}}"

# 观察状态
ros2 topic echo /delivery_status

# 人工确认装/卸货
ros2 service call /confirm_load std_srvs/srv/Trigger
ros2 service call /confirm_unload std_srvs/srv/Trigger

# 配送报告
ros2 service call /get_delivery_report delivery_interfaces/srv/GetDeliveryReport
```

## Environment

- **ROS distro**: Jazzy (`/opt/ros/jazzy`)
- **OS**: Ubuntu 24.04
- **Simulator**: Gazebo Sim (Harmonic) via `ros_gz_sim`
- **Robot**: TurtleBot3 Waffle Pi (`export TURTLEBOT3_MODEL=waffle_pi`)

## Code Style

- **C++**: 4-space indent, Doxygen 注释，`include/<package_name>/` 头文件组织
- **Python/YAML/launch**: 2-space indent
- **Package names**: lowercase with underscores
- **注释语言**: 中英文均可，同一文件保持一致

## Development Progress

- [x] Phase 1: 骨架 + 单点导航（delivery_manager 核心流程可用）
- [x] Phase 2: 行为树 + 停靠确认（delivery_executor BT 宿主 + 4 个 BT 叶节点）
- [x] Phase 3: 多订单 + 生命周期 + 工程化

> 核心功能骨架已实现。低电量时直接中止当前配送（demo 级别，不恢复电量），仿真默认使用自定义仓库场景。

详见 `plan/` 目录下的分阶段计划文档。

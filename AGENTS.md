# AGENTS.md

本文件供 Codex 在进入本仓库时快速初始化上下文。优先帮助后续会话理解“当前事实”，避免被过期计划误导。

## 信息来源优先级

1. 代码和 launch/config 文件
2. [README.md](/root/ros2_delivery_robot/README.md)
3. [CLAUDE.md](/root/ros2_delivery_robot/CLAUDE.md)
4. [suggestions.md](/root/ros2_delivery_robot/suggestions.md)
5. `plan/` 下的阶段文档

规则：
- `plan/` 目录更多是历史设计过程，不应默认视为当前实现。
- 如果 README、计划文档和代码冲突，以代码为准。
- `suggestions.md` 是 2026-03-19 的一次仓库审查记录，适合用来识别已知问题和文档漂移。

## 沟通约定

- 默认用中文与用户沟通。
- 代码评审优先指出问题、风险、行为回归和测试缺口。
- Git commit message 默认使用中文，技术术语可保留英文。

## 项目一句话

这是一个基于 ROS 2 Jazzy 的室内配送机器人 demo：接收订单，前往取货点等待人工装货确认，再前往送货点等待人工卸货确认，支持多订单队列、优先级调度和失败重试。

## 当前仓库结构

真实源码主要在 [ros2_ws/src](/root/ros2_delivery_robot/ros2_ws/src)：

- [delivery_interfaces](/root/ros2_delivery_robot/ros2_ws/src/delivery_interfaces)：自定义 msg/srv/action
- [delivery_core](/root/ros2_delivery_robot/ros2_ws/src/delivery_core)：核心运行逻辑
- [delivery_lifecycle](/root/ros2_delivery_robot/ros2_ws/src/delivery_lifecycle)：生命周期管理
- [delivery_bringup](/root/ros2_delivery_robot/ros2_ws/src/delivery_bringup)：launch、参数、RViz
- [delivery_simulation](/root/ros2_delivery_robot/ros2_ws/src/delivery_simulation)：地图、world、仿真资源

注意：
- 仓库里存在生成产物，例如 `ros2_ws/build`、`ros2_ws/install`、`ros2_ws/log`，以及 [delivery_core](/root/ros2_delivery_robot/ros2_ws/src/delivery_core) 下的包内 `build/install/log`。不要把它们当源码。

## 关键运行链路

- `delivery_manager` 是系统调度器：
  - 文件：[delivery_manager.hpp](/root/ros2_delivery_robot/ros2_ws/src/delivery_core/include/delivery_core/delivery_manager.hpp)
  - 文件：[delivery_manager.cpp](/root/ros2_delivery_robot/ros2_ws/src/delivery_core/src/delivery_manager.cpp)
  - 负责站点加载、系统就绪检查、订单接收、优先级队列、取消、报告、状态发布
  - 不直接调 Nav2，而是把单订单执行委托给 `delivery_executor`

- `delivery_executor` 是单订单执行器：
  - 文件：[delivery_executor.cpp](/root/ros2_delivery_robot/ros2_ws/src/delivery_core/src/delivery_executor.cpp)
  - 是 `LifecycleNode`
  - 对外提供 `ExecuteDelivery` action server
  - 内部持有 Nav2 `NavigateToPose` action client
  - 宿主 BehaviorTree，并提供 `/confirm_load`、`/confirm_unload`

- `delivery_lifecycle_manager` 负责把 `delivery_executor` 从 `Unconfigured` 推进到 `Active`：
  - 文件：[delivery_lifecycle_manager.cpp](/root/ros2_delivery_robot/ros2_ws/src/delivery_lifecycle/src/delivery_lifecycle_manager.cpp)

## 默认启动方式

- 一键入口：[demo.launch.py](/root/ros2_delivery_robot/ros2_ws/src/delivery_bringup/launch/demo.launch.py)
- 分阶段启动：
  1. [simulation.launch.py](/root/ros2_delivery_robot/ros2_ws/src/delivery_bringup/launch/simulation.launch.py)
  2. [navigation.launch.py](/root/ros2_delivery_robot/ros2_ws/src/delivery_bringup/launch/navigation.launch.py)
  3. [delivery.launch.py](/root/ros2_delivery_robot/ros2_ws/src/delivery_bringup/launch/delivery.launch.py)

`demo.launch.py` 使用定时延迟来避免 Gazebo、Nav2、AMCL、delivery 节点的启动竞争。

## BehaviorTree 事实

- 行为树 XML 在 [behavior_trees](/root/ros2_delivery_robot/ros2_ws/src/delivery_core/behavior_trees)
- 默认 bringup 加载的是 [single_delivery_robust.xml](/root/ros2_delivery_robot/ros2_ws/src/delivery_core/behavior_trees/single_delivery_robust.xml)
- [delivery_mission.xml](/root/ros2_delivery_robot/ros2_ws/src/delivery_core/behavior_trees/delivery_mission.xml) 才包含 `CheckBattery`

这点很重要：
- README 的 BT 说明比默认 demo 更“完整”
- 实际默认演示运行的是“带导航重试”的树，而不是“带电量前置检查”的树

## 常用命令

环境准备：

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
```

启动 demo：

```bash
ros2 launch delivery_bringup demo.launch.py
```

提交订单：

```bash
ros2 service call /submit_order delivery_interfaces/srv/SubmitOrder \
  "{order: {order_id: 'order_001', pickup_station: 'station_A', dropoff_station: 'station_C', priority: 0}}"
```

确认装卸货：

```bash
ros2 service call /confirm_load std_srvs/srv/Trigger
ros2 service call /confirm_unload std_srvs/srv/Trigger
```

测试：

```bash
cd ros2_ws
colcon test --packages-select delivery_core
colcon test-result --verbose
```

## 关键配置文件

- 站点配置：[stations.yaml](/root/ros2_delivery_robot/ros2_ws/src/delivery_bringup/config/stations.yaml)
- manager 参数：[delivery_manager.yaml](/root/ros2_delivery_robot/ros2_ws/src/delivery_bringup/config/delivery_manager.yaml)
- lifecycle 参数：[lifecycle_manager.yaml](/root/ros2_delivery_robot/ros2_ws/src/delivery_bringup/config/lifecycle_manager.yaml)
- 仓库地图：[warehouse.yaml](/root/ros2_delivery_robot/ros2_ws/src/delivery_simulation/maps/warehouse.yaml)
- 仓库 world：[warehouse.sdf](/root/ros2_delivery_robot/ros2_ws/src/delivery_simulation/worlds/warehouse.sdf)

## 当前已知问题与文档漂移

以下内容来自代码和 [suggestions.md](/root/ros2_delivery_robot/suggestions.md) 的交叉整理，后续处理问题时优先核对这些点：

- `plan/phase3_full_system.md` 已明显过期，不能当作当前状态说明书。
- `.github/workflows/build.yml` 只覆盖构建和 `delivery_core` 测试，没有覆盖 demo / launch / 仿真链路。
- [scripts/smoke_test.sh](/root/ros2_delivery_robot/scripts/smoke_test.sh) 是端到端冒烟入口，但据 2026-03-19 的审查记录，可能因 `set -u` 与 ROS setup 脚本交互而失效；在依赖它前应重新验证。
- [delivery_manager.yaml](/root/ros2_delivery_robot/ros2_ws/src/delivery_bringup/config/delivery_manager.yaml) 中的 `nav2_action_name` 目前看起来不是有效配置入口，修改前先确认代码是否真正读取它。
- `GetDeliveryReport` 的接口注释与实现顺序可能不完全一致；如果改报告逻辑，先核对服务定义和 `delivery_manager.cpp` 的返回顺序。
- `WaitForConfirmation` 的 `confirm_type` 容错较弱，修改 BT XML 时不要拼错字符串。

## 接手时的建议顺序

1. 先读 [README.md](/root/ros2_delivery_robot/README.md) 和本文件，建立运行链路概念
2. 再看 [delivery.launch.py](/root/ros2_delivery_robot/ros2_ws/src/delivery_bringup/launch/delivery.launch.py)、[delivery_manager.cpp](/root/ros2_delivery_robot/ros2_ws/src/delivery_core/src/delivery_manager.cpp)、[delivery_executor.cpp](/root/ros2_delivery_robot/ros2_ws/src/delivery_core/src/delivery_executor.cpp)
3. 涉及行为树时查看 `behavior_trees/*.xml` 与 `src/nodes/*.cpp`
4. 涉及系统状态、是否“已经实现”之类的问题时，优先核对代码，其次参考 `suggestions.md`，最后再看 `plan/`

## 判断准则

- 讨论“系统现在怎么跑”时，信代码和 launch
- 讨论“为什么这样设计”时，信 README 和 CLAUDE
- 讨论“下一步应该修什么”时，参考 suggestions
- 讨论“最初想做什么”时，再看 `plan/`

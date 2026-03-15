# 项目总览：ROS2 多点配送机器人

## 背景与动机

用户有一份 ROS2 机器人任务系统设计方案，但感觉当前方案是"技术点堆砌"，缺乏真实场景驱动。目标是构建一个**场景驱动**的求职展示项目，复用 `Ros2Learning` 仓库的成熟代码。

## 场景描述

**一句话：** 室内配送机器人接收配送订单，自主导航至取货点等待装货确认，再导航至送货点等待卸货确认，支持多订单队列和失败重试。

**典型应用：**
- 仓库内部物料配送（工位间转运零件）
- 医院药品/样本配送（药房→病房）
- 办公楼文件/快递配送（前台→工位）

**为什么可信：** 这是 Keenon、普渡科技、Bear Robotics 等公司的核心产品形态，面试官秒懂。

## 技术栈

| 项 | 选型 |
|---|---|
| 操作系统 | Ubuntu 24.04 |
| ROS 版本 | ROS2 Jazzy |
| 仿真器 | Gazebo Harmonic |
| 机器人 | TurtleBot3 Waffle Pi |
| 行为树 | BehaviorTree.CPP v4 |
| 导航 | Nav2 |
| 构建 | colcon / ament_cmake |

## 关键设计决策

1. **停靠等待确认 vs mock 机械臂** — 选择"等待确认"模式，因为这是真实配送机器人的实际做法（Keenon/普渡科技就是这样），比假装有个机械臂在抓取更可信。

2. **Action 驱动配送执行** — delivery_manager 通过 ExecuteDelivery.action 调用 delivery_executor，支持取消和进度反馈。

3. **BT 编排站点行为 + 状态机管理订单队列** — 各取所长：BT 擅长单次配送内的决策流程，状态机擅长订单级别的状态管理。

4. **5 个包精简结构** — 不设 perception 包（配送不需要视觉检测），不设 motion_manager（导航已封装在 BT 节点中）。

## 代码复用映射

| Ros2Learning 源码 | 新项目用途 | 改动量 |
|---|---|---|
| `task_runner.cpp` (607 LOC) | → delivery_manager：状态机重构为配送场景 | 中 |
| `nav2_client.cpp` (756 LOC) | → 导航底层，AMCL 初始化、TF、sim_time | 几乎不改 |
| `move_base_node.cpp` (BT节点) | → NavigateToStation BT 节点模板 | 小，改端口名 |
| `lifecycle_manager_node.cpp` (377 LOC) | → delivery_lifecycle | 小，扩展 managed_nodes |
| `custom_interfaces/` | → delivery_interfaces 模板 | 重新定义 |
| `composition/` 模式 | → 组件注册 + 零拷贝模式 | 模式复用 |
| `multithreading/` 模式 | → MultiThreadExecutor + CallbackGroup 隔离 | 模式复用 |
| `blocking_node.cpp` | → WaitForConfirmation 节点的阻塞等待模式参考 | 模式参考 |
| `tb3_spawner` | → 仿真环境 TurtleBot3 启动 | 直接参考 |

## 模块结构

```
ros2_delivery_robot/
  ros2_ws/src/
    delivery_interfaces/         # 自定义消息/服务/动作
    delivery_core/               # C++ 核心（任务编排 + 行为树 + BT 节点）
    delivery_lifecycle/          # 生命周期管理
    delivery_simulation/         # Gazebo 仿真环境资源
    delivery_bringup/            # Launch + 配置文件
```

## 项目亮点（面试谈资）

1. **真实产品形态** — 不是 ROS2 教程拼装，而是可以说清"解决什么问题"的完整系统
2. **Action-based 任务解耦** — 面试必考：什么时候用 Service vs Action
3. **BT + 状态机混合架构** — 展示对不同编排工具适用场景的理解
4. **Lifecycle 不是为了演示** — 有具体理由：Nav2 必须在 executor 之前就绪
5. **停靠等待确认** — 展示对真实配送机器人工作流程的理解

## 开发路线

分三个阶段，每阶段可独立演示。详见各阶段计划文档：

- `phase1_skeleton.md` — 骨架 + 单点导航 ✅ 已完成
- `phase2_behavior_tree.md` — 行为树 + 停靠确认
- `phase3_full_system.md` — 多订单 + 生命周期 + 工程化

## 验证方式

1. `ros2 launch delivery_bringup demo.launch.py` 一键启动全系统
2. `ros2 service call /submit_order` 提交配送订单
3. `ros2 topic echo /delivery_status` 观察状态流转
4. `ros2 service call /confirm_load` 和 `/confirm_unload` 模拟人工确认
5. `ros2 service call /get_delivery_report` 获取配送报告
6. 录制 2 分钟演示视频覆盖完整配送流程

# 全量代码审核报告

> 审核时间：2026-04-06
> 审核工具：Codex 深度审核 + 8 维度全量扫描
> 覆盖范围：delivery_interfaces / delivery_core / delivery_lifecycle / delivery_bringup

---

## 严重问题 (P0)

| # | 位置 | 问题 | 结论 |
|---|------|------|------|
| 1 | `navigate_to_station.cpp:86` | `result_ready_` 跨线程读写 | 复核后降级——`std::atomic<bool>` 默认 seq_cst，**安全** |
| 2 | `delivery_executor.cpp:663` | `battery_level_` load-modify-store 非原子 | 当前单任务语义安全，但设计脆弱，扩展性差 |
| 3 | `delivery_manager.cpp:620` | `handle_cancel_order` 嵌套三层锁 | 复核后当前无死锁，但**锁层级未文档化**，维护风险中等 |

---

## 高优问题 (P1) — Top 3 优先修复

### P1-1: NavigateToStation 重试场景竞态

- **文件**: `delivery_core/src/nodes/navigate_to_station.cpp:124`
- **问题**: 在 `RetryUntilSuccessful` 下，旧 `result_callback` 的 late write 可能让新导航被误判完成。`onStart()` 被再次调用时，`result_future_` 和 `goal_handle_` 被覆盖，但上一次的 `result_callback` lambda 仍会写入 `result_ready_`。如果旧回调在新 `onStart` 的 `result_ready_ = false` 之后到达，会导致新导航被误判完成。
- **影响**: 配送可靠性——重试场景下可能跳过实际导航
- **修复建议**: 添加 generation counter 或 goal_handle 匹配检查，只有当 goal_handle 匹配当前活跃 handle 时才设置 `result_ready_`

### P1-2: `bt_execution_thread_` 赋值未加锁

- **文件**: `delivery_core/src/delivery_executor.cpp:501`
- **问题**: `handle_accepted` 中 `bt_execution_thread_` 赋值在 action server 回调线程执行，而 `join_bt_thread()` 在 `execution_mutex_` 保护下读取它。赋值本身未在 `execution_mutex_` 保护范围内。
- **影响**: 节点 deactivate 与新 goal 并发时可能导致线程对象状态不一致
- **修复建议**: 将 `bt_execution_thread_` 赋值放入 `execution_mutex_` 保护范围

### P1-3: `finish_execution` lambda 按引用捕获 goal_handle

- **文件**: `delivery_core/src/delivery_executor.cpp:516`
- **问题**: `const auto finish_execution = [this, &goal_handle]() { ... };` 捕获了栈上 `shared_ptr` 的引用。当前同步调用安全，但极脆弱。
- **影响**: 如果未来改为异步调用会导致悬垂引用
- **修复建议**: 改为按值捕获 `[this, goal_handle]`

---

## 中优问题 (P2)

### P2-1: `stations_` 服务回调中无锁读取

- **文件**: `delivery_manager.cpp:508,515`
- **问题**: `stations_` 在 `run()` 阶段一次性加载后不再修改，`handle_submit_order` 在服务回调线程中读取它。虽然当前语义上只读安全，但缺乏形式化同步保证。
- **修复建议**: 添加注释说明 `stations_` 在 `system_ready_` 置 true 后不再修改的约定，或使用 `shared_mutex` 保护

### P2-2: 每次 tick 按值拷贝整个 StationMap

- **文件**: `navigate_to_station.cpp:41`
- **问题**: `auto stations = config().blackboard->get<StationMap>("stations");` 每次 tick 拷贝整个站点表
- **修复建议**: 改为引用获取或缓存指针

### P2-3: 默认 BT XML 未包含 CheckBattery

- **文件**: `single_delivery_robust.xml`
- **问题**: 默认树文件中没有 `CheckBattery` 节点，只有 `delivery_mission.xml` 包含。低电量时仍会尝试配送。
- **修复建议**: 将默认 tree_file 改为 `delivery_mission.xml`，或在 `single_delivery_robust.xml` 中添加 CheckBattery

### P2-4: 嵌套锁顺序需文档化

- **文件**: `delivery_manager.cpp:572-578`
- **问题**: `handle_submit_order` 中嵌套 `queue_mutex_` 和 `current_order_mutex_`，当前安全但锁层级未文档化
- **修复建议**: 在头文件中注释锁层级顺序

---

## 低优/建议 (P3)

### P3-1: `Pose2D`/`Station` 重复定义

- **文件**: `delivery_manager.hpp:155-160` 和 `navigate_to_station.hpp`
- **问题**: 两处独立定义 `Pose2D`、`Station`，字段一致但类型系统不互通
- **建议**: 提取到公共头文件 `delivery_core/types.hpp`

### P3-2: BT tick 频率 100Hz 偏高

- **文件**: `delivery_executor.cpp:567`
- **问题**: 100Hz tick 对 demo 级配送频率过高，feedback 以 100Hz 发布增加不必要网络开销
- **建议**: 降低到 10-30Hz

### P3-3: lifecycle_manager 硬编码 2s 延迟

- **文件**: `delivery_lifecycle_manager.cpp:241-242`
- **问题**: `std::this_thread::sleep_for(2s)` 是经验值
- **建议**: 改为参数化配置或服务就绪轮询机制

### P3-4: `cmd_vel_pub_` 未使用 LifecyclePublisher

- **文件**: `delivery_executor.cpp:108`
- **问题**: 作为 LifecycleNode 应使用 LifecyclePublisher，当前使用普通 publisher
- **建议**: 改为 LifecyclePublisher 或注释说明当前行为是有意为之

### P3-5: `StationInfo.msg` 未被实际使用

- **文件**: `delivery_interfaces/msg/StationInfo.msg`
- **问题**: 代码中从未使用，manager 和 executor 都使用内部 `Station` 结构体
- **建议**: 标注 reserved 或删除

### P3-6: `handle_cancel` 的 shared_ptr 按值传递

- **文件**: `delivery_executor.cpp:481`
- **问题**: `const std::shared_ptr<GoalHandleExecuteDelivery>` 按值传递拷贝引用计数
- **建议**: 改为 `const std::shared_ptr<GoalHandleExecuteDelivery> &`

---

## 模块健康度

| 模块 | 评分 | 说明 |
|------|------|------|
| delivery_interfaces | 9/10 | 定义清晰完整，StationInfo.msg 未使用是唯一瑕疵 |
| delivery_core (manager) | 8/10 | 锁设计整体正确，嵌套锁顺序需文档化，共享数据无形式化同步保证 |
| delivery_core (executor) | 7.5/10 | 生命周期管理到位，线程赋值缺锁保护、引用捕获需改进 |
| delivery_core (BT 节点) | **7/10** | NavigateToStation 重试竞态是最大风险点 |
| delivery_lifecycle | 9/10 | 执行器隔离和线程管理设计良好 |
| delivery_bringup | 8.5/10 | launch 文件和配置参数一致性好 |

---

## 修复优先级

1. **P1-1**: NavigateToStation 重试竞态 — 影响配送可靠性，修复成本低
2. **P1-2**: `bt_execution_thread_` 赋值加锁 — 修复成本极低（加一行锁）
3. **P2-3**: 默认 BT XML 启用 CheckBattery — 修复方式：改默认 tree_file

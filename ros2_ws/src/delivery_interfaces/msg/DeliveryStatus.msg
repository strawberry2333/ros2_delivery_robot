# =============================================================================
# DeliveryStatus.msg — 配送状态消息
# =============================================================================
# 用途：实时反馈当前配送任务的执行状态，供监控端和上层调度系统订阅。
# 使用场景：
#   - delivery_manager 在状态机每次跳转时，通过 /delivery_status 话题发布此消息
#   - RViz 或自定义 GUI 订阅此话题以可视化展示配送进度
#   - GetDeliveryReport 服务的响应中以数组形式返回多条此消息
#   - ExecuteDelivery action 的 Feedback 中复用此消息的核心字段
# 发布频率：状态变化时立即发布，导航过程中周期性发布（约 1Hz）
# =============================================================================

# 状态更新时间戳
# 使用 ROS 时钟（仿真模式下为仿真时间，实机模式下为系统时间）
# 可用于判断消息新旧、计算配送耗时等
builtin_interfaces/Time stamp

# 订单唯一标识，与 DeliveryOrder.msg 中的 order_id 一一对应
string order_id

# ---------------------------------------------------------------------------
# 状态枚举常量 — 配送状态机的全部状态
# 状态流转顺序（正常流程）：
#   IDLE → GOING_TO_PICKUP → WAITING_LOAD → GOING_TO_DROPOFF → WAITING_UNLOAD → COMPLETE
# 任意阶段均可能跳转至 FAILED（如导航失败、超时等）
# ---------------------------------------------------------------------------
uint8 STATE_IDLE=0                   # 空闲：订单已入队，等待调度执行
uint8 STATE_GOING_TO_PICKUP=1        # 前往取货点：机器人正在导航至 pickup_station
uint8 STATE_WAITING_LOAD=2           # 等待装货：已到达取货点，等待人工确认装货完成
uint8 STATE_GOING_TO_DROPOFF=3       # 前往送货点：机器人正在导航至 dropoff_station
uint8 STATE_WAITING_UNLOAD=4         # 等待卸货：已到达送货点，等待人工确认卸货完成
uint8 STATE_COMPLETE=5               # 配送完成：订单已成功完成全部流程
uint8 STATE_FAILED=6                 # 配送失败：发生不可恢复的错误，详见 error_msg
uint8 STATE_CANCELED=7               # 配送取消：订单在执行中被用户取消

# 当前配送状态
# 取值范围：0~6，对应上述枚举常量
uint8 state

# 当前关联的站点 ID
# 含义随 state 变化：
#   STATE_GOING_TO_PICKUP / STATE_WAITING_LOAD → 值为 pickup_station
#   STATE_GOING_TO_DROPOFF / STATE_WAITING_UNLOAD → 值为 dropoff_station
#   STATE_IDLE / STATE_COMPLETE / STATE_FAILED → 可能为空字符串
string current_station

# 配送阶段进度（非实时导航距离百分比）
# 取值范围：0.0 ~ 1.0，由 BT 的 ReportDeliveryStatus 节点在关键阶段设置
# 用于标识配送流程进展到哪个阶段，而非导航路径的精确完成比例
float32 progress

# 错误描述信息
# 仅当 state == STATE_FAILED 时有值，描述失败原因
# 正常状态下为空字符串
# 示例值："Navigation timeout", "Station not found: station_X"
string error_msg

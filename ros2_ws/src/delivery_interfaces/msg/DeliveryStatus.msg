# 配送状态消息
# 实时反馈当前配送任务的执行状态

builtin_interfaces/Time stamp      # 状态更新时间戳
string order_id                    # 订单唯一标识

# 状态枚举常量
uint8 STATE_IDLE=0
uint8 STATE_GOING_TO_PICKUP=1
uint8 STATE_WAITING_LOAD=2
uint8 STATE_GOING_TO_DROPOFF=3
uint8 STATE_WAITING_UNLOAD=4
uint8 STATE_COMPLETE=5
uint8 STATE_FAILED=6

uint8 state                        # 当前状态
string current_station             # 当前关联站点 ID
float32 progress                   # 进度百分比 (0.0 ~ 1.0)
string error_msg                   # 错误描述（仅 STATE_FAILED 时有值）

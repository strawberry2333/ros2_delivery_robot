# 配送订单消息
# 描述一个从取货点到送货点的配送任务

string order_id                    # 订单唯一标识
string pickup_station              # 取货站点 ID
string dropoff_station             # 送货站点 ID

# 优先级枚举常量
uint8 PRIORITY_NORMAL=0
uint8 PRIORITY_HIGH=1
uint8 PRIORITY_URGENT=2

uint8 priority 0                   # 优先级

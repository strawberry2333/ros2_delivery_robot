# 配送订单消息
# 描述一个从取货点到送货点的配送任务

string order_id                    # 订单唯一标识
string pickup_station              # 取货站点 ID
string dropoff_station             # 送货站点 ID
uint8 priority 0                   # 优先级 (0=普通, 1=高, 2=紧急)

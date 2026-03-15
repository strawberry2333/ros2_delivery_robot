# 站点信息消息
# 描述仓库/室内环境中的一个配送站点

string station_id                  # 站点唯一标识

# 站点在地图坐标系中的位姿
float64 x
float64 y
float64 yaw                        # 偏航角 (弧度)

# 站点类型
uint8 TYPE_PICKUP=0
uint8 TYPE_DROPOFF=1
uint8 TYPE_CHARGE=2

uint8 station_type                 # 站点类型

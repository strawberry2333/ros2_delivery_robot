# ============================================================================
# delivery_simulation — Gazebo 仿真资源包
# ============================================================================
# 本包为纯资源包（不含任何 C++/Python 可执行文件），仅负责将仿真所需的
# 静态资源（世界文件、地图、模型、RViz 配置等）安装到 ROS 2 的 share 目录，
# 供 launch 文件在运行时通过 FindPackageShare 定位和加载。
# ============================================================================

# 要求 CMake 最低版本 3.8，这是 ament_cmake 构建系统的基本要求
cmake_minimum_required(VERSION 3.8)

# 声明项目名称，同时也是 ROS 2 包名，
# colcon build --packages-select delivery_simulation 即依据此名称定位
project(delivery_simulation)

# 查找 ament_cmake 构建工具链（ROS 2 的 CMake 扩展），
# 提供 ament_package()、install() 等 ROS 2 专用宏和函数
find_package(ament_cmake REQUIRED)

# 将仿真资源目录安装到 install/delivery_simulation/share/delivery_simulation/ 下，
# 使其他包的 launch 文件可以通过以下方式引用：
#   FindPackageShare('delivery_simulation') + '/worlds/xxx.sdf'
#
# 各目录用途：
#   worlds — Gazebo 世界描述文件（.sdf/.world），定义仿真场景
#   maps   — 导航用静态地图文件（.pgm 栅格图 + .yaml 元数据），供 map_server 加载
#   models — Gazebo 自定义模型（如配送站台、障碍物等 SDF 模型）
#   rviz   — RViz2 可视化配置文件（.rviz），预设显示面板和话题订阅
#   config — 其他仿真相关配置（如 Gazebo 桥接参数等）
install(DIRECTORY
  worlds maps models rviz config
  DESTINATION share/${PROJECT_NAME}
)

# 生成 ament 包索引文件（package.xml 解析、环境钩子等），
# 必须在 CMakeLists.txt 末尾调用，确保包能被 ros2 pkg 命令和 launch 系统发现
ament_package()

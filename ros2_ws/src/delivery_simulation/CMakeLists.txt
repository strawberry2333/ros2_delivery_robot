# ============================================================================
# delivery_simulation — Gazebo 仿真资源包
# ============================================================================
# 本包不承担任何业务逻辑，角色是“场景与资源提供者”：
# - world 文件定义 Gazebo 仿真中的房间、货架、站点标记和物理环境
# - map 文件定义 Nav2/AMCL 使用的静态栅格地图
# - rviz 文件定义调试和演示时的可视化布局
# - models/config 则补充场景模型和桥接配置
#
# delivery_bringup 的 launch 文件会通过 FindPackageShare 定位这些资源，
# 这样 demo 在安装后仍然可以从 share 目录直接加载，而不是依赖源码路径。
# ============================================================================

# 要求 CMake 最低版本 3.8，这是 ament_cmake 构建系统的基本要求
cmake_minimum_required(VERSION 3.8)

# 声明项目名称，同时也是 ROS 2 包名，
# colcon build --packages-select delivery_simulation 即依据此名称定位
project(delivery_simulation)

# 查找 ament_cmake 构建工具链（ROS 2 的 CMake 扩展），
# 提供 ament_package()、install() 等 ROS 2 专用宏和函数
find_package(ament_cmake REQUIRED)

# 将仿真资源目录安装到 install/delivery_simulation/share/delivery_simulation/ 下。
# 这样 launch 文件只要知道包名，就能在运行时找到完整的场景资源。
#
# 目录职责如下：
#   worlds  - Gazebo 世界描述文件。这里决定“机器人在哪里、墙和货架怎么摆”。
#   maps    - 静态导航地图。这里决定“Nav2 和 AMCL 认为空间长什么样”。
#   models  - 可复用的仿真模型资源。后续扩展货架、标识牌时可继续放这里。
#   rviz    - 演示/调试用 RViz 配置。只负责显示，不影响真实运行逻辑。
#   config  - 其他仿真相关参数，如 bridge 配置等。
install(DIRECTORY
  worlds maps models rviz config
  DESTINATION share/${PROJECT_NAME}
)

# 生成 ament 包索引文件（package.xml 解析、环境钩子等），
# 必须在 CMakeLists.txt 末尾调用，确保包能被 ros2 pkg 命令和 launch 系统发现
ament_package()

# =============================================================================
# delivery_bringup 包的 CMake 构建配置
# =============================================================================
# 本包为纯配置/启动包，不包含可编译的 C++ 源码。
# 它的职责是把 launch、配置、RViz 资源安装到 share 目录，供上层 demo 统一调用。
# 构建命令：cd ros2_ws && colcon build --packages-select delivery_bringup --symlink-install
# =============================================================================

# CMake 最低版本要求，ROS2 Jazzy 推荐 3.8+
cmake_minimum_required(VERSION 3.8)

# 项目名称，必须与 package.xml 中的 <name> 一致
project(delivery_bringup)

# 查找 ament_cmake 构建工具包（ROS2 标准构建系统）
# 提供 ament_package()、install() 等宏和函数
find_package(ament_cmake REQUIRED)

# 安装 launch/config/rviz 资源到 share 目录，供 ros2 launch 和其他包查找。
# 安装后的路径：
#   install/delivery_bringup/share/delivery_bringup/launch/
#   install/delivery_bringup/share/delivery_bringup/config/
#   install/delivery_bringup/share/delivery_bringup/rviz/
# 其他包通过 get_package_share_directory("delivery_bringup") 读取这些资源。
# --symlink-install 模式下会创建符号链接，方便快速迭代 launch 和 YAML。
install(DIRECTORY
  launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

# 生成 ament 包索引文件，让 ros2 launch / colcon / ament_index 能发现本包资源。
# 这是纯资源包的“收口”步骤，必须放在 CMakeLists.txt 末尾。
ament_package()

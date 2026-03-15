# =============================================================================
# delivery_bringup 包的 CMake 构建配置
# =============================================================================
# 本包为纯配置/启动包（不包含 C++ 源码），仅安装 launch 文件和配置文件。
# 构建命令：cd ros2_ws && colcon build --packages-select delivery_bringup --symlink-install
# =============================================================================

# CMake 最低版本要求，ROS2 Jazzy 推荐 3.8+
cmake_minimum_required(VERSION 3.8)

# 项目名称，必须与 package.xml 中的 <name> 一致
project(delivery_bringup)

# 查找 ament_cmake 构建工具包（ROS2 标准构建系统）
# 提供 ament_package()、install() 等宏和函数
find_package(ament_cmake REQUIRED)

# 安装 launch 和 config 目录到包的 share 目录
# 安装后的路径：install/delivery_bringup/share/delivery_bringup/launch/
#              install/delivery_bringup/share/delivery_bringup/config/
# 其他包可通过 get_package_share_directory("delivery_bringup") 获取该路径
# --symlink-install 模式下会创建符号链接，修改源文件后无需重新构建
install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME}
)

# 生成 ament 包索引文件，使本包可被其他 ROS2 工具发现
# 必须在 CMakeLists.txt 末尾调用
ament_package()

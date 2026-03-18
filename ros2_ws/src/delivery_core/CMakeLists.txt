# =============================================================================
# delivery_core — CMakeLists.txt
# =============================================================================
# 本包是配送机器人系统的核心功能包，包含：
#   - delivery_manager 节点：订单队列管理 + Nav2 导航编排 + 装/卸货确认
# 构建前提：delivery_interfaces 必须先构建（colcon 自动处理依赖顺序）。
# 单独构建命令：
#   cd ros2_ws && colcon build --packages-select delivery_core --symlink-install
# =============================================================================

# CMake 最低版本要求（ROS2 Jazzy 推荐 3.8+）
cmake_minimum_required(VERSION 3.8)

# 项目名称，必须与 package.xml 中的 <name> 一致
project(delivery_core)

# ---------------------------------------------------------------------------
# 编译选项
# ---------------------------------------------------------------------------

# GCC / Clang 下开启严格警告，有助于在编译期发现潜在问题
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 强制使用 C++17 标准（rclcpp 和 nav2 API 依赖此标准）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------------------------------------------------------------------
# 依赖查找
# ---------------------------------------------------------------------------

# ROS2 构建基础设施
find_package(ament_cmake REQUIRED)

# ROS2 C++ 客户端库
find_package(rclcpp REQUIRED)

# ROS2 Action 客户端库（用于调用 Nav2 的 NavigateToPose action）
find_package(rclcpp_action REQUIRED)

# Nav2 消息定义（NavigateToPose action 等）
find_package(nav2_msgs REQUIRED)

# 几何消息（PoseStamped 等，用于构造导航目标点）
find_package(geometry_msgs REQUIRED)

# TF2 变换库（坐标系查询，用于获取机器人当前位置）
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 本项目自定义接口包（DeliveryOrder, DeliveryStatus, SubmitOrder 等）
find_package(delivery_interfaces REQUIRED)

# 标准服务（std_srvs/srv/Trigger，用于装/卸货确认）
find_package(std_srvs REQUIRED)

# YAML 解析库（用于读取 stations.yaml 站点配置文件）
find_package(yaml-cpp REQUIRED)

# BehaviorTree.CPP v4（Phase 2 行为树驱动配送执行）
find_package(behaviortree_cpp REQUIRED)

# ROS2 生命周期节点库（Phase 3 delivery_executor 转 LifecycleNode）
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)

# ---------------------------------------------------------------------------
# 构建目标 1：delivery_bt_nodes 静态库（BT 叶节点）
# ---------------------------------------------------------------------------
add_library(delivery_bt_nodes STATIC
  src/nodes/navigate_to_station.cpp
  src/nodes/dock_at_station.cpp
  src/nodes/wait_for_confirmation.cpp
  src/nodes/report_delivery_status.cpp
  src/nodes/check_battery.cpp
)

target_include_directories(delivery_bt_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(delivery_bt_nodes
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  std_srvs
  delivery_interfaces
  behaviortree_cpp
)

# ---------------------------------------------------------------------------
# 构建目标 2：delivery_manager 可执行文件
# ---------------------------------------------------------------------------
add_executable(delivery_manager
  src/delivery_manager.cpp          # 核心实现：订单管理 + 导航编排 + 状态发布
  src/delivery_manager_main.cpp     # 入口：MultiThreadedExecutor 启动
)

# 声明 ament 依赖（自动处理 include 路径和链接库）
ament_target_dependencies(delivery_manager
  rclcpp
  rclcpp_action
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  delivery_interfaces
)

# yaml-cpp 不是 ament 包，需要手动链接
target_link_libraries(delivery_manager yaml-cpp)

# 设置头文件搜索路径（BUILD_INTERFACE 用于构建时，INSTALL_INTERFACE 用于安装后）
target_include_directories(delivery_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ---------------------------------------------------------------------------
# 构建目标 3：delivery_executor 可执行文件（BT 宿主节点）
# ---------------------------------------------------------------------------
add_executable(delivery_executor
  src/delivery_executor.cpp
  src/delivery_executor_main.cpp
)

target_include_directories(delivery_executor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(delivery_executor
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  lifecycle_msgs
  nav2_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  std_srvs
  delivery_interfaces
  behaviortree_cpp
)

target_link_libraries(delivery_executor delivery_bt_nodes yaml-cpp)

# ---------------------------------------------------------------------------
# 安装规则
# ---------------------------------------------------------------------------

# 安装可执行文件到 lib/<package_name>/ 目录（ROS2 标准路径）
install(TARGETS
  delivery_manager
  delivery_executor
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件（供其他包依赖时使用）
install(DIRECTORY include/
  DESTINATION include
)

# 安装行为树 XML 文件到 share 目录
install(DIRECTORY behavior_trees/
  DESTINATION share/${PROJECT_NAME}/behavior_trees
)

# ---------------------------------------------------------------------------
# 测试配置
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # 跳过版权和 cpplint 检查（项目尚处于开发阶段）
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # CheckBattery 单元测试
  ament_add_gtest(test_check_battery test/test_check_battery.cpp)
  target_link_libraries(test_check_battery delivery_bt_nodes)
  ament_target_dependencies(test_check_battery behaviortree_cpp)

  # ReportDeliveryStatus 单元测试
  ament_add_gtest(test_report_delivery_status test/test_report_delivery_status.cpp)
  target_link_libraries(test_report_delivery_status delivery_bt_nodes)
  ament_target_dependencies(test_report_delivery_status
    rclcpp delivery_interfaces behaviortree_cpp)

  # DeliveryManager 服务级测试
  ament_add_gtest(test_delivery_manager_services
    test/test_delivery_manager_services.cpp
    src/delivery_manager.cpp)
  target_include_directories(test_delivery_manager_services PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  target_link_libraries(test_delivery_manager_services yaml-cpp)
  ament_target_dependencies(test_delivery_manager_services
    rclcpp rclcpp_action geometry_msgs tf2 tf2_ros tf2_geometry_msgs
    delivery_interfaces)

  # BT 装配测试
  ament_add_gtest(test_bt_assembly test/test_bt_assembly.cpp)
  target_link_libraries(test_bt_assembly delivery_bt_nodes)
  target_compile_definitions(test_bt_assembly PRIVATE
    BEHAVIOR_TREES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/behavior_trees")
  ament_target_dependencies(test_bt_assembly
    rclcpp rclcpp_action nav2_msgs geometry_msgs
    delivery_interfaces behaviortree_cpp)

  # 安装测试数据
  install(FILES test/test_stations.yaml
    DESTINATION share/${PROJECT_NAME}/test)
endif()

# ament 包注册（必须在 CMakeLists.txt 末尾调用）
ament_package()

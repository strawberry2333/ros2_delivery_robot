# =============================================================================
# delivery_interfaces — CMakeLists.txt
# =============================================================================
# 本包专门用于定义配送机器人系统的自定义 ROS2 接口（msg/srv/action）。
# 构建时 rosidl 工具链会自动为这些接口生成 C++ 和 Python 的绑定代码。
# 注意：本包必须在 delivery_core 等依赖包之前构建（colcon 会自动处理依赖顺序）。
# 单独构建命令：
#   cd ros2_ws && colcon build --packages-select delivery_interfaces --symlink-install
# =============================================================================

# CMake 最低版本要求（ROS2 Jazzy 推荐 3.8+）
cmake_minimum_required(VERSION 3.8)

# 项目名称，必须与 package.xml 中的 <name> 一致
project(delivery_interfaces)

# ---------------------------------------------------------------------------
# 依赖查找
# ---------------------------------------------------------------------------

# ament_cmake：ROS2 的 CMake 构建基础设施
find_package(ament_cmake REQUIRED)

# rosidl_default_generators：接口代码生成器（将 .msg/.srv/.action 编译为 C++/Python 代码）
find_package(rosidl_default_generators REQUIRED)

# builtin_interfaces：ROS2 内置基础类型（如 builtin_interfaces/Time），
# DeliveryStatus.msg 中的 stamp 字段依赖此包
find_package(builtin_interfaces REQUIRED)

# ---------------------------------------------------------------------------
# 接口文件注册 — 声明需要生成代码的所有 .msg / .srv / .action 文件
# ---------------------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  # 消息定义（Message）
  "msg/DeliveryOrder.msg"              # 配送订单消息
  "msg/DeliveryStatus.msg"             # 配送状态消息
  "msg/StationInfo.msg"                # 站点信息消息

  # 服务定义（Service）
  "srv/SubmitOrder.srv"                # 提交订单服务
  "srv/CancelOrder.srv"               # 取消订单服务
  "srv/GetDeliveryReport.srv"          # 获取配送报告服务

  # 动作定义（Action）
  "action/ExecuteDelivery.action"      # 执行单次配送动作（Phase 2）

  # DEPENDENCIES：声明接口文件中引用的外部消息包
  # 此处 builtin_interfaces 提供了 DeliveryStatus.msg 使用的 Time 类型
  DEPENDENCIES builtin_interfaces
)

# 导出运行时依赖，使下游包能正确找到本包生成的接口类型
ament_export_dependencies(rosidl_default_runtime)

# ament 包注册（必须在 CMakeLists.txt 末尾调用）
ament_package()

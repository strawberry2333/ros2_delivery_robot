# =============================================================================
# delivery_interfaces — CMakeLists.txt
# =============================================================================
# 本包专门用于定义配送机器人系统的自定义 ROS 2 接口（msg/srv/action）。
# 它本身不承载业务逻辑，只负责描述跨包通信契约，让 core / bringup /
# lifecycle 等包共享同一套消息、服务和 action 定义。
# 构建时 rosidl 工具链会自动为这些接口生成 C++ 和 Python 的绑定代码。
# 注意：本包必须在 delivery_core 等依赖包之前构建，colcon 会自动处理依赖顺序。
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

# rosidl_default_generators：接口代码生成器（将 .msg/.srv/.action 编译为 C++/Python 代码）。
# 没有它，消息定义只是一组文本文件，无法被下游节点 include 或 import。
find_package(rosidl_default_generators REQUIRED)

# builtin_interfaces：ROS 2 内置基础类型（如 builtin_interfaces/Time）。
# 本包的状态消息包含时间戳字段，因此必须显式依赖这个基础接口包。
find_package(builtin_interfaces REQUIRED)

# ---------------------------------------------------------------------------
# 接口文件注册：
# 这里列出所有需要生成代码的 .msg / .srv / .action 文件。
# rosidl 会根据这些声明生成头文件、Python 模块和类型支持代码。
# ---------------------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  # 消息定义（Message）
  # DeliveryOrder：描述“要送什么、送到哪、优先级多高”
  # DeliveryStatus：描述“当前送到哪一步了”
  # StationInfo：描述“站点在哪里、是什么类型”
  "msg/DeliveryOrder.msg"
  "msg/DeliveryStatus.msg"
  "msg/StationInfo.msg"

  # 服务定义（Service）
  # SubmitOrder：外部系统提交订单的入口
  # CancelOrder：取消队列中或执行中的订单
  # GetDeliveryReport：查询全局订单状态快照
  "srv/SubmitOrder.srv"
  "srv/CancelOrder.srv"
  "srv/GetDeliveryReport.srv"

  # 动作定义（Action）
  # ExecuteDelivery：由 manager 触发的单次配送任务，支持反馈和取消
  "action/ExecuteDelivery.action"

  # DEPENDENCIES：声明接口文件中引用的外部消息包
  # 此处 builtin_interfaces 提供了 DeliveryStatus.msg 使用的 Time 类型。
  # 这条依赖同时会进入生成器和下游接口类型支持代码。
  DEPENDENCIES builtin_interfaces
)

# 导出运行时依赖，使下游包在运行时能找到本包生成的接口类型支持。
ament_export_dependencies(rosidl_default_runtime)

# ---------------------------------------------------------------------------
# 测试配置
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ament 包注册，告诉构建系统这是一个标准 ament_cmake 接口包。
ament_package()

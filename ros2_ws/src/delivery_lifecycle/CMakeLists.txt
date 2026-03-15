# CMake 最低版本要求（ROS 2 Jazzy 推荐 3.8+）
cmake_minimum_required(VERSION 3.8)

# 项目名称，与 package.xml 中的 <name> 保持一致
project(delivery_lifecycle)

# 启用严格编译警告，尽早发现潜在问题
# -Wall: 启用常见警告  -Wextra: 启用额外警告  -Wpedantic: 严格遵循 C++ 标准
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ======================== 依赖查找 ========================
# ament_cmake: ROS 2 构建系统基础
find_package(ament_cmake REQUIRED)
# rclcpp: ROS 2 C++ 客户端库
find_package(rclcpp REQUIRED)
# rclcpp_components: 组件注册宏，支持 composition 模式动态加载
find_package(rclcpp_components REQUIRED)
# lifecycle_msgs: 生命周期服务消息定义（ChangeState, GetState）
find_package(lifecycle_msgs REQUIRED)

# 将所有依赖收集到变量中，方便后续统一引用
set(DEPS rclcpp rclcpp_components lifecycle_msgs)

# ======================== 构建共享库 ========================
# 构建为 SHARED 库（而非可执行文件），这是 ROS 2 组件化节点的标准做法，
# 使节点既可以独立运行，也可以通过 ComponentManager 动态加载
add_library(delivery_lifecycle_manager_component SHARED
  src/delivery_lifecycle_manager.cpp
)

# 设置头文件搜索路径
# BUILD_INTERFACE: 构建时使用源码目录下的 include/
# INSTALL_INTERFACE: 安装后使用安装目录下的 include/
target_include_directories(delivery_lifecycle_manager_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 链接所有 ROS 2 依赖（自动处理头文件路径和库链接）
ament_target_dependencies(delivery_lifecycle_manager_component ${DEPS})

# ======================== 组件注册 ========================
# 注册为 ROS 2 组件节点：
#   PLUGIN: 组件的完整类名（命名空间::类名）
#   EXECUTABLE: 同时生成一个独立可执行文件，方便直接 ros2 run 启动
# 这个宏会自动生成组件描述文件，供 ComponentManager 在运行时查找和加载
rclcpp_components_register_node(delivery_lifecycle_manager_component
  PLUGIN "delivery_lifecycle::DeliveryLifecycleManager"
  EXECUTABLE delivery_lifecycle_manager
)

# ======================== 安装规则 ========================
# 安装共享库到标准路径，供其他包链接或 ComponentManager 加载
# ARCHIVE: 静态库（.a）  LIBRARY: 共享库（.so）  RUNTIME: 可执行文件
install(TARGETS
  delivery_lifecycle_manager_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件到 include/ 目录，供其他包引用
install(DIRECTORY include/ DESTINATION include)

# ======================== 测试配置 ========================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 跳过版权和代码风格检查（项目自定义风格）
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  # 自动发现并添加 lint 测试依赖
  ament_lint_auto_find_test_dependencies()
endif()

# 生成 ament 包索引和配置文件，使本包可被其他 ROS 2 包发现和依赖
ament_package()

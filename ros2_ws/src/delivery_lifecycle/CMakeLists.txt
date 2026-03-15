cmake_minimum_required(VERSION 3.8)
project(delivery_lifecycle)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(lifecycle_msgs REQUIRED)

set(DEPS rclcpp rclcpp_components lifecycle_msgs)

add_library(delivery_lifecycle_manager_component SHARED
  src/delivery_lifecycle_manager.cpp
)

target_include_directories(delivery_lifecycle_manager_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(delivery_lifecycle_manager_component ${DEPS})

rclcpp_components_register_node(delivery_lifecycle_manager_component
  PLUGIN "delivery_lifecycle::DeliveryLifecycleManager"
  EXECUTABLE delivery_lifecycle_manager
)

install(TARGETS
  delivery_lifecycle_manager_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

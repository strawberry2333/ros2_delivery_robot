cmake_minimum_required(VERSION 3.8)
project(delivery_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DeliveryOrder.msg"
  "msg/DeliveryStatus.msg"
  "msg/StationInfo.msg"
  "srv/SubmitOrder.srv"
  "srv/CancelOrder.srv"
  "srv/GetDeliveryReport.srv"
  "action/ExecuteDelivery.action"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

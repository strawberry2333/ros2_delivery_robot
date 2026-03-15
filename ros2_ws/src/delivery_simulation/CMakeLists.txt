cmake_minimum_required(VERSION 3.8)
project(delivery_simulation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  worlds maps models rviz config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

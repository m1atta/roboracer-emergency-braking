cmake_minimum_required(VERSION 3.8)
project(safety_node)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)

add_executable(safety_node src/safety_node.cpp)
ament_target_dependencies(
  safety_node
  rclcpp
  sensor_msgs
  nav_msgs
  ackermann_msgs
)

install(TARGETS safety_node DESTINATION lib/${PROJECT_NAME})

ament_package()

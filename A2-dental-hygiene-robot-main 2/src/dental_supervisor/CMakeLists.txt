cmake_minimum_required(VERSION 3.8)
project(dental_supervisor)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(dental_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(supervisor_node src/supervisor_node.cpp)
target_compile_features(supervisor_node PUBLIC cxx_std_17)
ament_target_dependencies(supervisor_node
  rclcpp
  rclcpp_action
  builtin_interfaces
  dental_interfaces
  sensor_msgs
  std_msgs
)

install(TARGETS supervisor_node DESTINATION lib/${PROJECT_NAME})
ament_package()

cmake_minimum_required(VERSION 3.8)
project(dental_tool_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(dental_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(tool_control_node src/tool_control_node.cpp)
target_compile_features(tool_control_node PUBLIC cxx_std_17)
ament_target_dependencies(tool_control_node
  rclcpp
  rclcpp_action
  dental_interfaces
  std_msgs
)

install(TARGETS tool_control_node DESTINATION lib/${PROJECT_NAME})
ament_package()

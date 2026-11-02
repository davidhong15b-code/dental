cmake_minimum_required(VERSION 3.8)
project(dental_perception)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(dental_interfaces REQUIRED)

add_executable(perception_node src/perception_node.cpp)
target_compile_features(perception_node PUBLIC cxx_std_17)
ament_target_dependencies(perception_node
  rclcpp
  rclcpp_action
  sensor_msgs
  dental_interfaces
)

install(TARGETS perception_node DESTINATION lib/${PROJECT_NAME})
ament_package()

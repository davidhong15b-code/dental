cmake_minimum_required(VERSION 3.8)
project(dental_target_manager)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dental_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(target_manager_node src/target_manager_node.cpp)
target_compile_features(target_manager_node PUBLIC cxx_std_17)
ament_target_dependencies(target_manager_node
  rclcpp
  dental_interfaces
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)

install(TARGETS target_manager_node DESTINATION lib/${PROJECT_NAME})
ament_package()

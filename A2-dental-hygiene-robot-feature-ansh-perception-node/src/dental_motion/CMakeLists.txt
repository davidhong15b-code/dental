cmake_minimum_required(VERSION 3.8)
project(dental_motion)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(dental_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(motion_node src/motion_node.cpp)
target_compile_features(motion_node PUBLIC cxx_std_17)
ament_target_dependencies(motion_node
  rclcpp
  rclcpp_action
  dental_interfaces
  geometry_msgs
  std_msgs
  moveit_ros_planning_interface
)

add_executable(station_motion_node src/station_motion_node.cpp)
target_compile_features(station_motion_node PUBLIC cxx_std_17)
ament_target_dependencies(station_motion_node
  rclcpp rclcpp_action dental_interfaces geometry_msgs std_msgs
  tf2_ros moveit_ros_planning_interface)

# Standalone MoveIt validation utility. Does not change TreatTarget action behaviour.
add_executable(motion_smoke_test src/motion_smoke_test.cpp)
target_compile_features(motion_smoke_test PUBLIC cxx_std_17)
ament_target_dependencies(motion_smoke_test
  rclcpp
  geometry_msgs
  std_msgs
  moveit_ros_planning_interface
)

install(TARGETS motion_node station_motion_node motion_smoke_test DESTINATION lib/${PROJECT_NAME})
ament_package()

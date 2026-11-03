cmake_minimum_required(VERSION 3.8)
project(dental_bringup)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/mock_inspection_action.py
  scripts/apply_workcell.py
  DESTINATION lib/${PROJECT_NAME})
ament_package()

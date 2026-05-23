cmake_minimum_required(VERSION 3.8)
project(robot_explorer)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/frontier_detector
  scripts/explorer
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

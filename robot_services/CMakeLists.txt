cmake_minimum_required(VERSION 3.8)
project(robot_services)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/go_to_pose_server
  scripts/detect_object_server
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(robot_actions)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# ── NavigateToPose action server ─────────────────────────────────────────────
add_executable(NavigateToPose_action_server
  src/NavigateToPose_action_server.cpp
)
ament_target_dependencies(NavigateToPose_action_server
  rclcpp rclcpp_action nav2_msgs robot_interfaces
)

# ── SearchObject action server ───────────────────────────────────────────────
add_executable(SearchObject_action_server
  src/SearchObject_action_server.cpp
)
ament_target_dependencies(SearchObject_action_server
  rclcpp rclcpp_action robot_interfaces
  cv_bridge sensor_msgs vision_msgs
  geometry_msgs tf2_ros tf2_geometry_msgs
)
target_link_libraries(SearchObject_action_server
  ${OpenCV_LIBRARIES}
)

install(TARGETS
  NavigateToPose_action_server
  SearchObject_action_server
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

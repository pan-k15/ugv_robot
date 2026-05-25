cmake_minimum_required(VERSION 3.8)
project(robot_tasks)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)

# ── explore ───────────────────────────────────────────────────────────
add_executable(explore src/explore.cpp)
ament_target_dependencies(explore rclcpp sensor_msgs geometry_msgs)

# ── patrol ────────────────────────────────────────────────────────────
add_executable(patrol src/patrol.cpp)
ament_target_dependencies(patrol
  rclcpp sensor_msgs geometry_msgs visualization_msgs
  tf2_ros tf2_geometry_msgs)

# ── inspection ────────────────────────────────────────────────────────
add_executable(inspection src/inspection.cpp)
ament_target_dependencies(inspection
  rclcpp sensor_msgs geometry_msgs vision_msgs visualization_msgs
  tf2_ros tf2_geometry_msgs)

# ── patrol_v2 ─────────────────────────────────────────────────────────
add_executable(patrol_v2 src/patrol_v2.cpp)
ament_target_dependencies(patrol_v2
  rclcpp sensor_msgs geometry_msgs visualization_msgs
  tf2_ros tf2_geometry_msgs robot_interfaces)

install(TARGETS explore patrol inspection patrol_v2
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# ── Python task scripts ───────────────────────────────────────────────────────
install(PROGRAMS
  scripts/object_approach
  scripts/zone_object_hunt
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

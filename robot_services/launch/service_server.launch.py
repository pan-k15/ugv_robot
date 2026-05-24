"""
service_server.launch.py — starts all robot_services service servers.

Servers launched:
  /go_to_pose       (robot_interfaces/srv/GoToPose)
      Wraps Nav2 NavigateToPose action.  Requires Nav2 to be running.
  /detect_object    (robot_interfaces/srv/DetectObject)
      Finds a named YOLO class using detections + depth camera.
      Requires robot_vision yolo_node and a depth camera bridge.

Usage:
  ros2 launch robot_services service_server.launch.py
  ros2 launch robot_services service_server.launch.py use_sim_time:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use /clock from simulation instead of wall time',
    )

    go_to_pose_server = Node(
        package='robot_services',
        executable='go_to_pose_server',
        name='go_to_pose_server',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    detect_object_server = Node(
        package='robot_services',
        executable='detect_object_server',
        name='detect_object_server',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    return LaunchDescription([
        use_sim_time_arg,
        go_to_pose_server,
        detect_object_server,
    ])

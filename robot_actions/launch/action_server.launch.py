"""
action_server.launch.py — starts all robot_actions action servers.

Servers launched:
  /robot_navigate_to_pose   (robot_interfaces/action/NavigateToPose)
      Wraps Nav2 NavigateToPose with distance-remaining feedback.
      Requires Nav2 to be running.

  /search_object            (robot_interfaces/action/SearchObject)
      Rotates the robot 360° while scanning with YOLO + depth camera.
      Requires robot_vision (vision.launch.py) to be running.

Usage:
  ros2 launch robot_actions action_server.launch.py
  ros2 launch robot_actions action_server.launch.py use_sim_time:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use /clock from simulation instead of wall time',
        ),

        Node(
            package='robot_actions',
            executable='NavigateToPose_action_server',
            name='navigate_to_pose_action_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        Node(
            package='robot_actions',
            executable='SearchObject_action_server',
            name='search_object_action_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])

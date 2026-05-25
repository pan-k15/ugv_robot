"""
zone_object_hunt.launch — multi-zone search for a target object.

Navigates to each zone with /robot_navigate_to_pose, then runs /search_object
at that zone.  Stops as soon as the object is found.

Usage examples
--------------
# Three-zone search for a bottle:
  ros2 launch robot_tasks zone_object_hunt.launch.py \\
    target_class:=bottle \\
    zones_x:="[1.0, 4.0, 4.0]" \\
    zones_y:="[0.0, 0.0, 3.0]"

# Custom confidence and arrival headings:
  ros2 launch robot_tasks zone_object_hunt.launch.py \\
    target_class:=person \\
    min_confidence:=0.7 \\
    zones_x:="[2.0, 5.0]" \\
    zones_y:="[1.0, 3.0]" \\
    zones_yaw:="[0.0, 1.57]"

Prerequisites
-------------
  robot_actions  action_server.launch.py   (/robot_navigate_to_pose, /search_object)
  robot_vision   vision.launch.py          (YOLO detections)
  Nav2 + SLAM / localization
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'target_class', default_value='person',
            description='YOLO class label to search for'),
        DeclareLaunchArgument(
            'min_confidence', default_value='0.5',
            description='Minimum YOLO detection confidence [0–1]'),
        DeclareLaunchArgument(
            'zones_x', default_value='[0.0, 2.0, 4.0]',
            description='X coordinates of search zones in map frame'),
        DeclareLaunchArgument(
            'zones_y', default_value='[0.0, 2.0, 0.0]',
            description='Y coordinates of search zones in map frame'),
        DeclareLaunchArgument(
            'zones_yaw', default_value='[0.0, 0.0, 0.0]',
            description='Arrival heading (rad) at each zone'),

        Node(
            package='robot_tasks',
            executable='zone_object_hunt',
            name='zone_object_hunt',
            output='screen',
            parameters=[{
                'use_sim_time':   LaunchConfiguration('use_sim_time'),
                'target_class':   LaunchConfiguration('target_class'),
                'min_confidence': LaunchConfiguration('min_confidence'),
                'zones_x':        LaunchConfiguration('zones_x'),
                'zones_y':        LaunchConfiguration('zones_y'),
                'zones_yaw':      LaunchConfiguration('zones_yaw'),
            }],
        ),
    ])

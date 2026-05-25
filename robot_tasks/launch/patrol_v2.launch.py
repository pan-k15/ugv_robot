"""
patrol_v2.launch — inspection patrol with per-waypoint object detection.

At each waypoint the robot calls /detect_object for every class in
target_classes and logs the results.  A full checklist is printed when
the route is complete.

Usage examples
--------------
# Default square, looking for a fire extinguisher:
  ros2 launch robot_tasks patrol_v2.launch.py

# Custom waypoints, two target classes:
  ros2 launch robot_tasks patrol_v2.launch.py \
    waypoints_x:="[0.0, 4.0, 4.0, 0.0]" \
    waypoints_y:="[0.0, 0.0, 4.0, 4.0]" \
    target_classes:="[fire extinguisher, stop sign]"

Prerequisites
-------------
  robot_services service_server.launch.py   (/detect_object)
  robot_vision   vision.launch.py           (YOLO detections)
  robot_slam     slam.launch.py  (or localization + map)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',    default_value='true'),
        DeclareLaunchArgument('waypoints_x',     default_value='[0.0, 2.0, 2.0, 0.0]'),
        DeclareLaunchArgument('waypoints_y',     default_value='[0.0, 0.0, 2.0, 2.0]'),
        DeclareLaunchArgument('waypoints_yaw',   default_value='[0.0, 1.57, 3.14, -1.57]'),
        DeclareLaunchArgument('frame_id',        default_value='odom'),
        DeclareLaunchArgument('forward_speed',   default_value='0.2'),
        DeclareLaunchArgument('turn_speed',      default_value='0.6'),
        DeclareLaunchArgument('goal_tolerance',  default_value='0.3'),
        DeclareLaunchArgument('safety_dist',     default_value='0.45'),
        DeclareLaunchArgument('wait_time',       default_value='1.5'),
        DeclareLaunchArgument('loop',            default_value='false'),
        DeclareLaunchArgument('target_classes',
          default_value='[fire extinguisher]',
          description='Comma-separated YOLO class labels to check at each waypoint'),
        DeclareLaunchArgument('scan_wait',       default_value='0.5',
          description='Seconds to let the robot settle before calling /detect_object'),

        Node(
            package='robot_tasks',
            executable='patrol_v2',
            name='patrol_v2',
            output='screen',
            parameters=[{
                'use_sim_time':   LaunchConfiguration('use_sim_time'),
                'waypoints_x':    LaunchConfiguration('waypoints_x'),
                'waypoints_y':    LaunchConfiguration('waypoints_y'),
                'waypoints_yaw':  LaunchConfiguration('waypoints_yaw'),
                'frame_id':       LaunchConfiguration('frame_id'),
                'forward_speed':  LaunchConfiguration('forward_speed'),
                'turn_speed':     LaunchConfiguration('turn_speed'),
                'goal_tolerance': LaunchConfiguration('goal_tolerance'),
                'safety_dist':    LaunchConfiguration('safety_dist'),
                'wait_time':      LaunchConfiguration('wait_time'),
                'loop':           LaunchConfiguration('loop'),
                'target_classes': LaunchConfiguration('target_classes'),
                'scan_wait':      LaunchConfiguration('scan_wait'),
            }],
        ),
    ])

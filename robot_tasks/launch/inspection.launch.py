"""
Inspection launch — drives the robot to each waypoint, performs a 360° scan
to detect objects with YOLO, then reports all sightings.

Example override:
  ros2 launch robot_tasks inspection.launch.py \
    waypoints_x:="[0.0, 3.0, 3.0, 0.0]" \
    waypoints_y:="[0.0, 0.0, 3.0, 3.0]"
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('waypoints_x',     default_value='[0.0, 2.0, 2.0, 0.0]'),
        DeclareLaunchArgument('waypoints_y',     default_value='[0.0, 0.0, 2.0, 2.0]'),
        DeclareLaunchArgument('frame_id',        default_value='odom'),
        DeclareLaunchArgument('forward_speed',   default_value='0.2'),
        DeclareLaunchArgument('turn_speed',      default_value='0.6'),
        DeclareLaunchArgument('scan_turn_speed', default_value='0.8'),
        DeclareLaunchArgument('goal_tolerance',  default_value='0.3'),
        DeclareLaunchArgument('safety_dist',     default_value='0.45'),
        DeclareLaunchArgument('scan_duration',   default_value='8.0'),
        DeclareLaunchArgument('camera_fov_deg',  default_value='90.0'),
        DeclareLaunchArgument('img_width',       default_value='640.0'),
        DeclareLaunchArgument('loop',            default_value='false'),

        Node(
            package='robot_tasks',
            executable='inspection',
            name='inspection',
            output='screen',
            parameters=[{
                'waypoints_x':     LaunchConfiguration('waypoints_x'),
                'waypoints_y':     LaunchConfiguration('waypoints_y'),
                'frame_id':        LaunchConfiguration('frame_id'),
                'forward_speed':   LaunchConfiguration('forward_speed'),
                'turn_speed':      LaunchConfiguration('turn_speed'),
                'scan_turn_speed': LaunchConfiguration('scan_turn_speed'),
                'goal_tolerance':  LaunchConfiguration('goal_tolerance'),
                'safety_dist':     LaunchConfiguration('safety_dist'),
                'scan_duration':   LaunchConfiguration('scan_duration'),
                'camera_fov_deg':  LaunchConfiguration('camera_fov_deg'),
                'img_width':       LaunchConfiguration('img_width'),
                'loop':            LaunchConfiguration('loop'),
                'use_sim_time':    True,
            }],
        ),
    ])

"""
Patrol launch — drives the robot around a square by default.

Override waypoints from the command line:
  ros2 launch robot_tasks patrol.launch.py \
    waypoints_x:="[0.0, 3.0, 3.0, 0.0]" \
    waypoints_y:="[0.0, 0.0, 3.0, 3.0]"
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('waypoints_x',    default_value='[0.0, 2.0, 2.0, 0.0]'),
        DeclareLaunchArgument('waypoints_y',    default_value='[0.0, 0.0, 2.0, 2.0]'),
        DeclareLaunchArgument('waypoints_yaw',  default_value='[0.0, 1.57, 3.14, -1.57]'),
        DeclareLaunchArgument('frame_id',       default_value='odom'),
        DeclareLaunchArgument('forward_speed',  default_value='0.2'),
        DeclareLaunchArgument('turn_speed',     default_value='0.6'),
        DeclareLaunchArgument('goal_tolerance', default_value='0.3'),
        DeclareLaunchArgument('safety_dist',    default_value='0.45'),
        DeclareLaunchArgument('wait_time',      default_value='1.5'),
        DeclareLaunchArgument('loop',           default_value='true'),

        Node(
            package='robot_tasks',
            executable='patrol',
            name='patrol',
            output='screen',
            parameters=[{
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
                'use_sim_time':   True,
            }],
        ),
    ])

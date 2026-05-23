"""
AMCL localization with a pre-built map.

Usage (standalone):
  ros2 launch robot_navigation localization_launch.py map:=/path/to/map.yaml

Or included from navigation.launch.py when map_yaml_file is provided.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('robot_navigation')
    nav2_params = os.path.join(pkg, 'config', 'nav2_params.yaml')

    map_yaml_file = LaunchConfiguration('map')
    use_sim_time  = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            description='Full path to map .yaml file to load'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock'),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                nav2_params,
                {'use_sim_time': use_sim_time,
                 'yaml_filename': map_yaml_file},
            ],
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time,
                 'autostart': True,
                 'node_names': ['map_server', 'amcl']},
            ],
        ),
    ])

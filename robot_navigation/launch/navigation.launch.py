"""
Nav2 navigation stack for the UGV Rover PT.

Two localization modes:
  • SLAM mode  (default) — slam_toolbox provides /map live.
    Run robot_slam first:
      ros2 launch robot_slam slam.launch.py use_sim_time:=true

  • Map mode — load a pre-built map and use AMCL.
    ros2 launch robot_navigation navigation.launch.py \
      map_yaml_file:=/path/to/map.yaml

Optional EKF odometry fusion (robot_localization):
    use_ekf:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                             IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('robot_navigation')
    nav2_params   = os.path.join(pkg, 'config', 'nav2_params.yaml')
    ekf_params    = os.path.join(pkg, 'config', 'ekf_params.yaml')
    loc_launch    = os.path.join(pkg, 'launch', 'localization_launch.py')

    map_yaml_file = LaunchConfiguration('map_yaml_file')
    use_sim_time  = LaunchConfiguration('use_sim_time')
    use_ekf       = LaunchConfiguration('use_ekf')

    # ── Declare arguments ────────────────────────────────────────────────
    declare_map = DeclareLaunchArgument(
        'map_yaml_file', default_value='',
        description='Path to map .yaml (empty = SLAM provides /map live)')
    declare_sim = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use Gazebo simulation clock')
    declare_ekf = DeclareLaunchArgument(
        'use_ekf', default_value='false',
        description='Launch EKF to fuse /odom + /imu/data')

    # ── AMCL localization (only when map_yaml_file is set) ───────────────
    localization = GroupAction(
        condition=IfCondition(
            PythonExpression(["'", map_yaml_file, "' != ''"])),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(loc_launch),
                launch_arguments={
                    'map': map_yaml_file,
                    'use_sim_time': use_sim_time,
                }.items(),
            ),
        ],
    )

    # ── EKF (optional) ───────────────────────────────────────────────────
    ekf_node = Node(
        condition=IfCondition(use_ekf),
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params, {'use_sim_time': use_sim_time}],
    )

    # ── Nav2 navigation stack ────────────────────────────────────────────
    def nav2_node(pkg_name, executable, name=None):
        return Node(
            package=pkg_name,
            executable=executable,
            name=name or executable,
            output='screen',
            parameters=[nav2_params, {'use_sim_time': use_sim_time}],
        )

    nav2_nodes = [
        nav2_node('nav2_controller',        'controller_server'),
        nav2_node('nav2_smoother',           'smoother_server'),
        nav2_node('nav2_planner',            'planner_server'),
        nav2_node('nav2_behaviors',          'behavior_server'),
        nav2_node('nav2_bt_navigator',       'bt_navigator'),
        nav2_node('nav2_waypoint_follower',  'waypoint_follower'),
        nav2_node('nav2_velocity_smoother',  'velocity_smoother'),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'smoother_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                    'waypoint_follower',
                    'velocity_smoother',
                ],
            }],
        ),
    ]

    return LaunchDescription([
        declare_map,
        declare_sim,
        declare_ekf,
        localization,
        ekf_node,
        *nav2_nodes,
    ])

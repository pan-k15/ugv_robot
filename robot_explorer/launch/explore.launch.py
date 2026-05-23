"""
Autonomous SLAM exploration launch.

Starts (in order):
  1. slam_toolbox  – builds the /map from /scan
  2. frontier_detector – RRT frontier detection on /map → /detected_points
  3. explorer      – drives toward nearest frontier, avoids obstacles

Prerequisites (started separately):
  ros2 launch robot_simulation gazebo.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    # ── Arguments ─────────────────────────────────────────────────────
    rrt_iters_arg = DeclareLaunchArgument(
        'rrt_iterations', default_value='600',
        description='Number of RRT iterations per detection cycle')

    step_arg = DeclareLaunchArgument(
        'step_size', default_value='0.5',
        description='RRT step size in metres')

    forward_speed_arg = DeclareLaunchArgument(
        'forward_speed', default_value='0.20',
        description='Exploration forward speed m/s')

    safety_arg = DeclareLaunchArgument(
        'safety_dist', default_value='0.45',
        description='Obstacle safety distance in metres')

    # ── SLAM toolbox (async, online mapping) ──────────────────────────
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('robot_slam'), 'launch', 'slam.launch.py'])
        ),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    # ── Frontier detector ─────────────────────────────────────────────
    frontier_detector = Node(
        package='robot_explorer',
        executable='frontier_detector',
        name='frontier_detector',
        output='screen',
        parameters=[{
            'use_sim_time':    True,
            'rrt_iterations':  LaunchConfiguration('rrt_iterations'),
            'step_size':       LaunchConfiguration('step_size'),
            'cluster_radius':  0.8,
            'detect_period':   2.0,
        }],
    )

    # ── Explorer (goal selection + navigation) ────────────────────────
    explorer = Node(
        package='robot_explorer',
        executable='explorer',
        name='explorer',
        output='screen',
        parameters=[{
            'use_sim_time':   True,
            'forward_speed':  LaunchConfiguration('forward_speed'),
            'turn_speed':     0.6,
            'goal_tolerance': 0.5,
            'safety_dist':    LaunchConfiguration('safety_dist'),
            'align_threshold': 0.20,
            'front_half_deg': 30.0,
            'dedup_radius':   0.5,
        }],
    )

    return LaunchDescription([
        rrt_iters_arg,
        step_arg,
        forward_speed_arg,
        safety_arg,
        slam,
        frontier_detector,
        explorer,
    ])

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use /clock from simulation'),
        DeclareLaunchArgument(
            'target_class', default_value='person',
            description='YOLO class label to search for'),
        DeclareLaunchArgument(
            'min_confidence', default_value='0.5',
            description='Minimum detection confidence [0–1]'),

        Node(
            package='robot_tasks',
            executable='object_approach',
            name='object_approach',
            output='screen',
            parameters=[{
                'use_sim_time':   LaunchConfiguration('use_sim_time'),
                'target_class':   LaunchConfiguration('target_class'),
                'min_confidence': LaunchConfiguration('min_confidence'),
            }],
        ),
    ])

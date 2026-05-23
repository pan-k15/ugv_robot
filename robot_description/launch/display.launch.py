from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('robot_description')

    urdf_path = PathJoinSubstitution([pkg, 'urdf', 'robot.urdf.xacro'])

    urdf_arg = DeclareLaunchArgument(
        'urdf',
        default_value=urdf_path,
        description='Absolute path to robot URDF/xacro file',
    )
    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Start joint_state_publisher_gui for interactive joint sliders',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock',
    )

    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('urdf')]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('use_gui')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    return LaunchDescription([
        urdf_arg,
        use_gui_arg,
        use_sim_time_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz,
    ])

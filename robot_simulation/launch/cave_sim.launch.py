from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_simulation = FindPackageShare('robot_simulation')

    default_world = PathJoinSubstitution([pkg_simulation, 'worlds', 'cave_world.sdf'])

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Absolute path to the Gazebo Sim SDF world file',
    )
    # Robot spawns at the south entrance of the cave, facing north (+Y).
    # Entry tunnel: x∈[-1,1], y∈[-10,-4].  Spawn at y=-8 (mid-tunnel).
    x_arg   = DeclareLaunchArgument('x',   default_value='0.0',    description='Spawn X (m)')
    y_arg   = DeclareLaunchArgument('y',   default_value='0.0',   description='Spawn Y (m)')
    z_arg   = DeclareLaunchArgument('z',   default_value='0.0',    description='Spawn Z (m)')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='1.5708', description='Spawn yaw (rad) — 1.5708 = facing north')
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Open RViz alongside Gazebo Sim',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use Gazebo simulation clock',
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_simulation, 'launch', 'gazebo.launch.py'])
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'x':    LaunchConfiguration('x'),
            'y':    LaunchConfiguration('y'),
            'z':    LaunchConfiguration('z'),
            'yaw':  LaunchConfiguration('yaw'),
            'rviz': LaunchConfiguration('rviz'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    return LaunchDescription([
        world_arg,
        x_arg, y_arg, z_arg, yaw_arg,
        rviz_arg,
        use_sim_time_arg,
        gazebo,
    ])

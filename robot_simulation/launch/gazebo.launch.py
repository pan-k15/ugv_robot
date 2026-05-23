from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_description = FindPackageShare('robot_description')
    pkg_simulation  = FindPackageShare('robot_simulation')
    pkg_ros_gz_sim  = FindPackageShare('ros_gz_sim')

    urdf_path    = PathJoinSubstitution([pkg_description, 'urdf', 'robot.urdf.xacro'])
    default_world = PathJoinSubstitution([pkg_simulation, 'worlds', 'empty.sdf'])

    # ── Launch arguments ──────────────────────────────────────────────
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Absolute path to the Gazebo Sim SDF world file',
    )
    x_arg   = DeclareLaunchArgument('x',   default_value='0.0',  description='Spawn X (m)')
    y_arg   = DeclareLaunchArgument('y',   default_value='0.0',  description='Spawn Y (m)')
    z_arg   = DeclareLaunchArgument('z',   default_value='0.0',  description='Spawn Z (m)')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0',  description='Spawn yaw (rad)')
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Open RViz alongside Gazebo Sim',
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([pkg_simulation, 'rviz', 'simulation.rviz']),
        description='RViz config file to use when rviz:=true',
    )

    # ── Gazebo Sim  (-r = start running immediately) ──────────────────
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={
            'gz_args': [LaunchConfiguration('world'), ' -r'],
        }.items(),
    )

    # ── Robot state publisher ─────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', urdf_path]), value_type=str
            ),
            'use_sim_time': True,
        }],
    )

    # ── Spawn robot from the /robot_description topic ─────────────────
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name',  'ugv_rover_pt',
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw'),
        ],
    )

    # ── GZ ↔ ROS 2 topic bridge ───────────────────────────────────────
    #   Syntax: /ros_topic@ros_type[gz_type   (GZ → ROS)
    #           /ros_topic@ros_type]gz_type   (ROS → GZ)
    #           /ros_topic@ros_type@gz_type   (bidirectional)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            # simulation clock
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # drive: cmd_vel ROS→GZ, odom + tf GZ→ROS
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            # joint states
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            # IMU
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            # camera info
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/depth_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/front_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            # depth point cloud
            '/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        parameters=[{'use_sim_time': True}],
    )

    # ── LiDAR bridge → /scan_raw (frame_id is empty from gz, relay fixes it) ──
    scan_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='scan_bridge',
        output='screen',
        arguments=['/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
        remappings=[('/scan', '/scan_raw')],
        parameters=[{'use_sim_time': True}],
    )

    scan_relay = Node(
        package='robot_simulation',
        executable='scan_relay',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'restamp': True,
        }],
    )

    # ── Image bridges ─────────────────────────────────────────────────
    # gz camera sensors publish to <topic>/image in gz namespace.
    # parameter_bridge maps each gz topic to the ROS /image_raw name.
    # gz camera sensors publish images to the bare <topic> name (no /image suffix).
    # camera_info and points DO use suffixes and are handled in the main bridge.
    rgb_image_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='rgb_image_bridge',
        output='screen',
        arguments=[
            '/camera@sensor_msgs/msg/Image[gz.msgs.Image',
            '/front_camera@sensor_msgs/msg/Image[gz.msgs.Image',
        ],
        remappings=[
            ('/camera',       '/camera/image_raw'),
            ('/front_camera', '/front_camera/image_raw'),
        ],
        parameters=[{'use_sim_time': True}],
    )

    depth_image_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='depth_image_bridge',
        output='screen',
        arguments=['/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image'],
        remappings=[('/depth_camera', '/depth_camera/image_raw')],
        parameters=[{'use_sim_time': True}],
    )

    # ── RViz (optional) ───────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        # gz-transport uses multicast for peer discovery.
        # GZ_IP=127.0.0.1 binds all gz nodes to the loopback interface so
        # the simulation works without an active LAN/WiFi link.
        SetEnvironmentVariable('GZ_IP', '127.0.0.1'),
        world_arg,
        x_arg, y_arg, z_arg, yaw_arg,
        rviz_arg,
        rviz_config_arg,
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        bridge,
        scan_bridge,
        scan_relay,
        rgb_image_bridge,
        depth_image_bridge,
        rviz,
    ])

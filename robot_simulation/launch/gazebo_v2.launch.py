"""
gazebo_v2.launch.py
Launches robot_v2 (ros2_control skid-steer) in Gazebo Harmonic.

Key differences vs gazebo.launch.py:
  - Uses robot_v2.urdf.xacro (ros2_control hardware interface)
  - Drive, odometry, and /joint_states come from ros2_control controllers,
    NOT from Gazebo plugins  →  those topics are removed from the gz bridge.
  - Three controller spawners are started after the robot is in the world:
      joint_state_broadcaster  (publishes /joint_states)
      diff_drive_controller    (subscribes /cmd_vel, publishes /odom + /tf)
      pan_tilt_controller      (JointTrajectoryController for pan/tilt)
  - Two topic relays keep the user-facing topics (/cmd_vel, /odom) intact
    so nav2 / teleoperation nodes do not need reconfiguration.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
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

    urdf_path    = PathJoinSubstitution([pkg_description, 'urdf', 'robot_v2.urdf.xacro'])
    default_world = PathJoinSubstitution([pkg_simulation, 'worlds', 'empty.sdf'])

    # ── Launch arguments ──────────────────────────────────────────────────
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Absolute path to the Gazebo Sim SDF world file',
    )
    x_arg   = DeclareLaunchArgument('x',   default_value='0.0', description='Spawn X (m)')
    y_arg   = DeclareLaunchArgument('y',   default_value='0.0', description='Spawn Y (m)')
    z_arg   = DeclareLaunchArgument('z',   default_value='0.0', description='Spawn Z (m)')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn yaw (rad)')
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

    # ── Gazebo Sim ────────────────────────────────────────────────────────
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={
            'gz_args': [LaunchConfiguration('world'), ' -r'],
        }.items(),
    )

    # ── Robot state publisher ─────────────────────────────────────────────
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

    # ── Spawn robot ───────────────────────────────────────────────────────
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

    # ── ros2_control: controller spawner ──────────────────────────────────
    # A flat TimerAction fires 20 s after launch start.  This is more
    # reliable than OnProcessExit(spawn_robot)+TimerAction because:
    #   1. spawn_robot exits as soon as Gazebo ACKs the spawn request,
    #      which is BEFORE gz_ros2_control has finished loading the YAML
    #      and registering the controller types on the CM.
    #   2. A nested TimerAction inside RegisterEventHandler can be silently
    #      skipped in some launch versions.
    # 20 s gives Gazebo Harmonic enough time to start, spawn the model,
    # run the gz_ros2_control plugin, load the YAML, and register types.
    # --controller-manager-timeout 30 is extra insurance in case the CM
    # comes up just after the spawner starts.
    # Do NOT pass --param-file (causes RCLInvalidROSArgsError in the CM).
    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            'diff_drive_controller',
            'pan_tilt_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
        ],
    )

    spawn_controllers = TimerAction(
        period=20.0,
        actions=[controller_spawner],
    )

    # ── Topic bridges (ros2_control ↔ standard nav topics) ────────────────
    # In Jazzy, diff_drive_controller dropped use_stamped_vel and now only
    # subscribes to ~/cmd_vel (TwistStamped).  cmd_vel_relay converts the
    # plain Twist on /cmd_vel → TwistStamped on /diff_drive_controller/cmd_vel
    # so nav2, teleop, and EKF nodes don't need reconfiguration.
    cmd_vel_relay = Node(
        package='robot_simulation',
        executable='cmd_vel_relay',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    odom_relay = Node(
        package='topic_tools',
        executable='relay',
        name='odom_relay',
        output='screen',
        arguments=['/diff_drive_controller/odom', '/odom'],
        parameters=[{'use_sim_time': True}],
    )

    # ── GZ ↔ ROS 2 bridge (sensors only) ─────────────────────────────────
    # Drive, odometry, /tf, and /joint_states are now owned by ros2_control
    # and published directly on ROS topics — no bridge entries needed for them.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            # simulation clock
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
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

    # ── LiDAR bridge + frame_id relay ────────────────────────────────────
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
            'restamp': False,
        }],
    )

    # ── Image bridges ─────────────────────────────────────────────────────
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

    # ── RViz ─────────────────────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        SetEnvironmentVariable('GZ_IP', '127.0.0.1'),
        world_arg,
        x_arg, y_arg, z_arg, yaw_arg,
        rviz_arg,
        rviz_config_arg,
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        spawn_controllers,   # TimerAction: fires spawner 20 s after launch start
        cmd_vel_relay,
        odom_relay,
        bridge,
        scan_bridge,
        scan_relay,
        rgb_image_bridge,
        depth_image_bridge,
        rviz,
    ])

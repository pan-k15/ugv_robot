# UGV Rover PT — ROS 2 Workspace

ROS 2 packages for the **Waveshare UGV Rover PT** (Raspberry Pi 4B / Pi 5 kit).  
Covers the full model description, Gazebo Harmonic simulation, and online SLAM mapping.

---

## Packages

| Package | Status | Purpose |
|---|---|---|
| `robot_description` | active | Xacro URDF model + RViz visualisation launch |
| `robot_simulation` | active | Gazebo Harmonic simulation, bridge, worlds |
| `robot_slam` | active | Online mapping with slam_toolbox |
| `robot_interfaces` | scaffold | Custom message / service / action definitions |
| `robot_navigation` | scaffold | Nav2 integration (planned) |
| `robot_services` | scaffold | Custom ROS 2 service nodes (planned) |
| `robot_tasks` | scaffold | High-level task logic (planned) |
| `robot_vision` | scaffold | Camera / perception nodes (planned) |

---

## Robot Overview

The UGV Rover PT is a 6-wheel skid-steer ground vehicle (~3.5 kg, 252 × 230 × 94 mm) with a pan-tilt camera arm and deck-mounted sensor suite.

```
                       ┌─────────────┐
                       │  Deck Plate │ ← LiDAR (rear-right)
  ┌──────────────────────────────────────────┐
  │  [FL]        arm_column                  │
  │              pan_link (±90° Z)           │
  │              tilt_link (−30°…+60° Y)     │
  │              camera / depth / rail       │
  │  [ML]                              [MR]  │ ← mid bogie (passive)
  │                                          │
  │  [RL]                              [RR]  │
  └──────────────────────────────────────────┘
  front_camera_link (126 mm stereo bar, front face)
```

### Drive

Skid-steer via the Gazebo `DiffDrive` plugin.  
Front and rear wheel pairs are driven; mid bogie wheels are passive.

| Property | Value |
|---|---|
| Wheel radius | 41.87 mm |
| Wheel separation (track) | ~178 mm |
| Ground clearance | 25.13 mm |

### Sensors

| Sensor | Frame | Topic | Rate |
|---|---|---|---|
| Deck LiDAR (GPU) | `lidar_link` | `/scan` | 10 Hz |
| Pan-tilt RGB camera | `camera_optical_frame` | `/camera/image_raw` | 30 Hz |
| Depth / IR sensor | `depth_optical_frame` | `/depth_camera/image_raw`, `/depth_camera/points` | 30 Hz |
| Front stereo bar | `front_camera_optical_frame` | `/front_camera/image_raw` | 30 Hz |
| IMU | `imu_link` | `/imu/data` | 100 Hz |

### Joints

| Joint | Type | Axis | Limits |
|---|---|---|---|
| `pan_joint` | revolute | Z (yaw) | ±90° |
| `tilt_joint` | revolute | Y (pitch) | −30° … +60° |
| `*_wheel_joint` (×6) | continuous | Y | — |

---

## Prerequisites

- ROS 2 **Jazzy** (or Humble with Gazebo Garden)
- Gazebo **Harmonic** (`gz-harmonic`)
- `ros-$ROS_DISTRO-ros-gz-*` bridge packages
- `slam-toolbox`

```bash
sudo apt install \
  ros-$ROS_DISTRO-ros-gz-sim \
  ros-$ROS_DISTRO-ros-gz-bridge \
  ros-$ROS_DISTRO-ros-gz-image \
  ros-$ROS_DISTRO-robot-state-publisher \
  ros-$ROS_DISTRO-joint-state-publisher-gui \
  ros-$ROS_DISTRO-slam-toolbox \
  ros-$ROS_DISTRO-rviz2 \
  ros-$ROS_DISTRO-xacro
```

---

## Build

```bash
cd ~/Documents/projects/robot/p12/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Launch

### Visualise in RViz (no simulation)

```bash
ros2 launch robot_description display.launch.py
```

| Argument | Default | Description |
|---|---|---|
| `urdf` | `robot.urdf.xacro` | Path to alternative URDF/Xacro |
| `use_gui` | `true` | Show joint-slider GUI |

---

### Gazebo simulation — empty world

```bash
ros2 launch robot_simulation gazebo.launch.py
```

| Argument | Default | Description |
|---|---|---|
| `world` | `empty.sdf` | Path to SDF world file |
| `x`, `y`, `z` | `0 0 0.15` | Spawn position (m) |
| `yaw` | `0.0` | Spawn heading (rad) |
| `rviz` | `true` | Open RViz alongside Gazebo |

---

### Gazebo simulation — obstacle world

10 × 10 m enclosed arena with perimeter walls, interior wall segments, cylinder pillars, and box obstacles — designed to give SLAM distinctive loop-closure features.

```bash
ros2 launch robot_simulation obstacle_sim.launch.py
```

Accepts the same arguments as `gazebo.launch.py`.

---

### Online SLAM mapping

Run alongside either simulation launch.

```bash
# Terminal 1 — simulation
ros2 launch robot_simulation obstacle_sim.launch.py rviz:=false

# Terminal 2 — SLAM
ros2 launch robot_slam slam.launch.py

# Terminal 3 — drive
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/cmd_vel
```

| Argument | Default | Description |
|---|---|---|
| `use_sim_time` | `true` | Set `false` on a real robot |
| `slam_params_file` | `robot_slam/config/slam.yaml` | Override SLAM parameters |

Verify SLAM is running:

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 topic echo /map --once
```

---

## ROS 2 Topics (simulation)

| Topic | Type | Direction |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | ROS → GZ |
| `/odom` | `nav_msgs/Odometry` | GZ → ROS |
| `/tf` | `tf2_msgs/TFMessage` | GZ → ROS |
| `/joint_states` | `sensor_msgs/JointState` | GZ → ROS |
| `/scan` | `sensor_msgs/LaserScan` | GZ → ROS (via relay) |
| `/imu/data` | `sensor_msgs/Imu` | GZ → ROS |
| `/camera/image_raw` | `sensor_msgs/Image` | GZ → ROS |
| `/depth_camera/image_raw` | `sensor_msgs/Image` | GZ → ROS |
| `/depth_camera/points` | `sensor_msgs/PointCloud2` | GZ → ROS |
| `/front_camera/image_raw` | `sensor_msgs/Image` | GZ → ROS |
| `/clock` | `rosgraph_msgs/Clock` | GZ → ROS |
| `/map` | `nav_msgs/OccupancyGrid` | slam_toolbox → ROS |

---

## TF Tree

```
map                           ← published by slam_toolbox
└── odom
    └── base_footprint
        └── base_link
            ├── front_left_wheel / front_right_wheel
            ├── rear_left_wheel  / rear_right_wheel
            ├── mid_left_wheel   / mid_right_wheel
            ├── front_camera_link → front_camera_optical_frame
            ├── deck_plate
            │   ├── lidar_link
            │   └── arm_column
            │       └── pan_link
            │           └── tilt_link
            │               ├── camera_link → camera_optical_frame
            │               ├── depth_sensor_link → depth_optical_frame
            │               └── rail_link
            └── imu_link
```

---

## License

Apache-2.0 — see each package's `LICENSE` file.

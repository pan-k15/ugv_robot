# AGENTS.md — AI Agent Guide

This file tells AI coding agents everything they need to orient quickly in this repo.

---

## Repository layout

```
ros2_ws/
├── src/                          ← git root (you are here)
│   ├── robot_description/
│   │   ├── urdf/robot.urdf.xacro ← single-file robot model (all geometry + plugins)
│   │   ├── launch/display.launch.py
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   └── robot_simulation/
│       ├── launch/gazebo.launch.py
│       ├── worlds/empty.sdf
│       ├── CMakeLists.txt
│       └── package.xml
├── build/                        ← colcon build output (not in git)
├── install/                      ← colcon install tree (not in git)
└── log/                          ← colcon logs (not in git)
```

Only `src/` is tracked by git.

---

## Build system

- **ROS 2** (Jazzy or Humble) + **Gazebo Harmonic**
- Build tool: `colcon`
- Package build type: `ament_cmake` (both packages are install-only; no compiled C++)

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` means launch files and URDFs are symlinked; edits to `src/` take effect without rebuilding.

---

## Key file: `robot_description/urdf/robot.urdf.xacro`

This is the only robot model file. All geometry, inertia, joints, sensor definitions, and Gazebo plugins live here.

### Structure (sections by comment header)

| Section | Content |
|---|---|
| 0. CONSTANTS | All physical dimensions as `<xacro:property>` — edit here first |
| 1. MATERIALS | Named colours used in visuals |
| 2. INERTIA MACROS | `box_inertia`, `cyl_inertia` — generate `<inertial>` blocks |
| 3. WHEEL MACROS | `wheel_link`, `wheel_joint` — reused for all 6 wheels |
| 4–7. Links/joints | `base_footprint` → `base_link` → wheels |
| 8–17. Sensors & arm | Deck plate, LiDAR, arm column, pan/tilt, cameras, IMU |
| 18. Gazebo plugins | Friction, DiffDrive, JointStatePublisher, sensor plugins |

### Coordinate convention (ROS REP-103)

- X = forward, Y = left, Z = up
- All joint origins are expressed in the **parent** link's frame
- `base_link` is at the **chassis centroid** (72 mm above ground)
- `base_footprint` is the ground-plane projection (z = 0)

### Critical dimension derivations

```
chassis_cog_z = ground_clearance + chassis_h/2  = 0.02513 + 0.04687 = 0.07200 m
wheel_z_bl    = wheel_r - chassis_cog_z          = 0.04187 - 0.07200 = -0.03013 m
wheel_x_bl    = chassis_l/2 - (wheel_r - side_overhang/2)
wheel_y_bl    = total_w/2 - wheel_len/2
```

Values marked `[EST]` in the file are estimates from photos — they can be refined if better data is available.

---

## Launch files

### `robot_description/launch/display.launch.py`

Nodes started: `robot_state_publisher`, `joint_state_publisher_gui` (optional), `rviz2`.  
No Gazebo. Used for offline model inspection.

### `robot_simulation/launch/gazebo.launch.py`

Nodes started:
1. `gz_sim` — Gazebo Harmonic world
2. `robot_state_publisher` — publishes `/robot_description` and `/tf` (static)
3. `ros_gz_sim/create` — spawns the robot model from `/robot_description`
4. `ros_gz_bridge/parameter_bridge` — bridges all non-image topics
5. `ros_gz_image/image_bridge` — bridges image topics (transport-aware)
6. `rviz2` — optional visualisation

**Bridge direction syntax:**
- `[gz_type` = GZ → ROS
- `]gz_type` = ROS → GZ
- `@gz_type` = bidirectional

---

## Active topics and their Gazebo source

| ROS topic | Gazebo plugin | Sensor/link |
|---|---|---|
| `/cmd_vel` | `DiffDrive` | — |
| `/odom` | `DiffDrive` | — |
| `/tf` | `DiffDrive` + `JointStatePublisher` | — |
| `/joint_states` | `JointStatePublisher` | all wheel + arm joints |
| `/scan` | `gpu_lidar` sensor | `lidar_link` |
| `/imu/data` | `imu` sensor | `imu_link` |
| `/camera/image_raw` | `camera` sensor | `camera_link` |
| `/depth_camera/image_raw` | `depth_camera` sensor | `depth_sensor_link` |
| `/depth_camera/points` | `depth_camera` sensor | `depth_sensor_link` |
| `/front_camera/image_raw` | `camera` sensor | `front_camera_link` |

---

## Common tasks

### Add a new sensor

1. Add `<xacro:property>` constants in section 0.
2. Add `<link>` + `<joint>` in the appropriate numbered section.
3. Add a `<gazebo reference="..."><sensor ...>` block in section 18.
4. Add the bridge argument to `gazebo.launch.py` (`bridge` node arguments).
5. If it's an image topic, add it to the `image_bridge` arguments too.

### Change robot dimensions

Edit the `<xacro:property>` values in section 0 of `robot.urdf.xacro`. All dependent geometry recalculates automatically via xacro math.

### Add a new world

1. Place the `.sdf` file in `robot_simulation/worlds/`.
2. Pass it at launch: `ros2 launch robot_simulation gazebo.launch.py world:=/abs/path/to/world.sdf`

### Add a new package

Create it under `src/`, add `robot_description` or `robot_simulation` as `exec_depend` in `package.xml` if it needs the model, then rebuild.

---

## What to avoid

- **Do not** modify files under `build/` or `install/` — they are generated.
- **Do not** add a second URDF file; the single `robot.urdf.xacro` is the source of truth.
- **Do not** hardcode install paths — use `FindPackageShare` in launch files.
- **Do not** use `ros2 pkg prefix` paths in launch files; prefer `PathJoinSubstitution`.
- When editing the Gazebo bridge arguments, keep `parameter_bridge` for all non-image topics and `image_bridge` for image topics — mixing them causes silent failures.

---

## Verification commands

```bash
# Check URDF parses cleanly
xacro src/robot_description/urdf/robot.urdf.xacro | check_urdf

# List all links
xacro src/robot_description/urdf/robot.urdf.xacro | grep '<link name'

# Inspect active topics after launching simulation
ros2 topic list
ros2 topic hz /scan
ros2 topic hz /imu/data
```

---

## Package dependencies

```
robot_simulation
└── robot_description   (exec_depend)
    └── xacro, robot_state_publisher, joint_state_publisher_gui, rviz2, ros_gz_sim
robot_simulation also depends on:
    ros_gz_sim, ros_gz_bridge, ros_gz_image, robot_state_publisher, xacro, rviz2
```

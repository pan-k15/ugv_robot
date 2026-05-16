# AGENTS.md — AI Agent Guide

This file tells AI coding agents everything they need to orient quickly in this repo.

---

## Repository layout

```
ros2_ws/
├── src/                              ← git root (you are here)
│   ├── robot_description/
│   │   ├── urdf/robot.urdf.xacro    ← single-file robot model
│   │   ├── launch/display.launch.py
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── robot_simulation/
│   │   ├── launch/
│   │   │   ├── gazebo.launch.py     ← main sim launch (all worlds)
│   │   │   └── obstacle_sim.launch.py ← wraps gazebo.launch.py, default: obstacles.sdf
│   │   ├── worlds/
│   │   │   ├── empty.sdf
│   │   │   └── obstacles.sdf        ← 10×10 m arena with walls, pillars, boxes
│   │   ├── scripts/
│   │   │   └── scan_relay           ← Python executable: fixes empty LiDAR frame_id
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── robot_slam/
│   │   ├── config/slam.yaml         ← slam_toolbox parameters
│   │   ├── launch/slam.launch.py    ← lifecycle slam_toolbox node
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── robot_interfaces/            ← scaffold (custom msgs/srvs/actions)
│   ├── robot_navigation/            ← scaffold (Nav2 integration)
│   ├── robot_services/              ← scaffold (custom service nodes)
│   ├── robot_tasks/                 ← scaffold (high-level task logic)
│   └── robot_vision/                ← scaffold (camera / perception)
├── build/                           ← colcon output (not in git)
├── install/                         ← colcon install tree (not in git)
└── log/                             ← colcon logs (not in git)
```

Only `src/` is tracked by git. Scaffold packages contain only `CMakeLists.txt`, `package.xml`, and `LICENSE`.

---

## Build system

- **ROS 2** (Jazzy or Humble) + **Gazebo Harmonic**
- Build tool: `colcon`
- Package build type: `ament_cmake` (all packages are install-only; no compiled C++)

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` means launch files, configs, and scripts are symlinked; edits to `src/` take effect without rebuilding. **Exception**: adding a new `install(PROGRAMS ...)` entry requires a rebuild to register the new executable.

---

## Key file: `robot_description/urdf/robot.urdf.xacro`

The only robot model file. All geometry, inertia, joints, sensor definitions, and Gazebo plugins live here.

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
lidar centre  = chassis_cog_z + chassis_h/2 + deck_h/2 + lidar_h/2  ≈ 0.144 m
```

Values marked `[EST]` are estimated from photos — refine if better data is available.

---

## Launch files

### `robot_description/launch/display.launch.py`

Nodes: `robot_state_publisher`, `joint_state_publisher_gui` (optional), `rviz2`.  
No Gazebo. Used for offline model inspection.

### `robot_simulation/launch/gazebo.launch.py`

The canonical simulation launcher. Always use this as the base.

Nodes started:
1. `gz_sim` — Gazebo Harmonic world (`-r` flag starts it immediately)
2. `robot_state_publisher` — publishes `/robot_description` and static TF
3. `ros_gz_sim/create` — spawns robot from `/robot_description`
4. `ros_gz_bridge/parameter_bridge` (`bridge`) — all non-image, non-scan topics
5. `ros_gz_bridge/parameter_bridge` (`scan_bridge`) — LiDAR only, remapped to `/scan_raw`
6. `robot_simulation/scan_relay` — reads `/scan_raw`, stamps `frame_id='lidar_link'`, publishes `/scan`
7. `ros_gz_image/image_bridge` — image topics (transport-aware, separate from parameter_bridge)
8. `rviz2` — optional

**Why the scan goes through a relay:** `gz_frame_id` in the URDF sensor XML is not respected by the `gpu_lidar` sensor in some gz-harmonic builds. The bridge publishes `/scan` with an empty `header.frame_id`, which prevents slam_toolbox from processing scans. The relay fixes this unconditionally (only stamps when empty).

**Bridge direction syntax:**
- `[gz_type` = GZ → ROS
- `]gz_type` = ROS → GZ
- `@gz_type` = bidirectional

**Do not** put image topics in `parameter_bridge` — they must go through `image_bridge` or they will fail silently.

### `robot_simulation/launch/obstacle_sim.launch.py`

Thin wrapper around `gazebo.launch.py` that sets `default_value` for `world` to `obstacles.sdf`. Accepts and forwards all the same arguments. Add new world-specific launchers using this same pattern.

### `robot_slam/launch/slam.launch.py`

Runs `async_slam_toolbox_node` as a **lifecycle node**. With `autostart:=true` (default), it self-configures and activates on startup. Subscribes to `/scan`, publishes `map→odom` TF and `/map`.

Key parameters forwarded to the node:
- `slam_params_file` — defaults to `robot_slam/config/slam.yaml`
- `use_sim_time` — default `true`; set `false` on real hardware

---

## Active topics and their source

| ROS topic | Source | Notes |
|---|---|---|
| `/cmd_vel` | ROS → GZ via `DiffDrive` | |
| `/odom` | GZ `DiffDrive` → ROS | |
| `/tf` | GZ `DiffDrive` + `JointStatePublisher` → ROS | |
| `/joint_states` | GZ `JointStatePublisher` → ROS | pan, tilt, all wheels |
| `/scan_raw` | GZ `gpu_lidar` → `scan_bridge` | intermediate; empty frame_id |
| `/scan` | `scan_relay` | frame_id stamped to `lidar_link` |
| `/imu/data` | GZ `imu` sensor → `bridge` | |
| `/camera/image_raw` | GZ `camera` sensor → `image_bridge` | |
| `/depth_camera/image_raw` | GZ `depth_camera` → `image_bridge` | |
| `/depth_camera/points` | GZ `depth_camera` → `bridge` | |
| `/front_camera/image_raw` | GZ `camera` → `image_bridge` | |
| `/map` | `slam_toolbox` | only when slam.launch.py is running |

---

## `robot_slam/config/slam.yaml`

slam_toolbox parameter file. Key settings to know:

| Parameter | Value | Why |
|---|---|---|
| `scan_topic` | `/scan` | matches relay output |
| `base_frame` | `base_footprint` | matches DiffDrive child frame |
| `odom_frame` | `odom` | matches DiffDrive frame_id |
| `max_laser_range` | `12.0` | matches LiDAR sensor max |
| `ceres_loss_function` | `'None'` | must be quoted string; unquoted YAML `None` is null and breaks Ceres config |
| `mode` | `mapping` | change to `localization` to use a saved map |

---

## `robot_simulation/worlds/obstacles.sdf`

10 × 10 m arena (0.15 m thick walls, 0.8 m tall) with:
- 4 perimeter walls
- 3 interior wall segments (L-shape + short divider)
- 4 cylinder pillars (r = 0.15 m) — curved surfaces give distinctive LiDAR signatures
- 3 box clusters of varying sizes and orientations
- 2 thin posts (0.1 × 0.1 m) — tests narrow-obstacle detection

All obstacles are ≥ 0.5 m tall, above the LiDAR centre height (~14 cm).  
Robot spawns at origin with clear surroundings.

---

## Common tasks

### Add a new sensor

1. Add `<xacro:property>` constants in section 0 of `robot.urdf.xacro`.
2. Add `<link>` + `<joint>` in the appropriate numbered section.
3. Add a `<gazebo reference="..."><sensor ...>` block in section 18.
4. For non-image, non-LiDAR sensors: add to the `bridge` node arguments in `gazebo.launch.py`.
5. For image sensors: add to the `image_bridge` arguments.
6. For a LiDAR-type sensor (gpu_lidar): follow the `scan_relay` pattern — bridge to a `_raw` topic, add a relay node that stamps the correct `frame_id`.

### Change robot dimensions

Edit `<xacro:property>` values in section 0. All dependent geometry recalculates via xacro math expressions.

### Add a new world

1. Place the `.sdf` file in `robot_simulation/worlds/`.
2. Create a launcher following `obstacle_sim.launch.py` — declare all args, pass them through to `gazebo.launch.py` via `IncludeLaunchDescription`, and set the new world as `default_value` for `world`.
3. No `CMakeLists.txt` change needed — `worlds/` is already installed.

### Add a new package

1. `ros2 pkg create --build-type ament_cmake <name>` under `src/`.
2. Add `exec_depend` entries for any packages it depends on.
3. Follow the install pattern of the closest existing package.

### Switch SLAM to localisation mode

1. Save the map first: `ros2 run nav2_map_server map_saver_cli -f /tmp/my_map`
2. In `robot_slam/config/slam.yaml`, set `mode: localization` and uncomment `map_file_name`.

---

## Known quirks

- **`gz_frame_id` ignored for `gpu_lidar`**: Some gz-harmonic builds do not apply `<gz_frame_id>` to GPU LiDAR sensors, leaving `header.frame_id` empty in the bridged LaserScan. The `scan_relay` node works around this. If a future gz-sim version fixes this, the relay becomes a no-op (it only stamps when frame_id is empty).
- **`ceres_loss_function: None`**: In YAML, bare `None` is parsed as null, not the string `"None"`. slam_toolbox's Ceres solver config requires a string. Always quote it: `'None'`.
- **Lifecycle node startup**: `slam_toolbox` runs as a lifecycle node. With `autostart:=true` it self-activates. If it appears stuck, check `ros2 lifecycle get /slam_toolbox` — it should reach `active`. If it's stuck in `configuring`, the params file is likely missing or malformed.
- **`--symlink-install` and new scripts**: Adding a new `install(PROGRAMS ...)` entry requires a full rebuild even with `--symlink-install`, because the symlink itself must be created.

---

## What to avoid

- **Do not** modify files under `build/` or `install/` — they are generated.
- **Do not** add a second URDF file; the single `robot.urdf.xacro` is the source of truth.
- **Do not** hardcode install paths — use `FindPackageShare` + `PathJoinSubstitution`.
- **Do not** put LiDAR or image topics in the main `bridge` node — use the relay pattern for LiDAR and `image_bridge` for cameras.
- **Do not** write world-specific simulation logic into `gazebo.launch.py` — wrap it with a thin launcher like `obstacle_sim.launch.py` instead.

---

## Verification commands

```bash
# Check URDF parses cleanly
xacro src/robot_description/urdf/robot.urdf.xacro | check_urdf

# Confirm scan frame_id is non-empty after launching sim
ros2 topic echo /scan --field header.frame_id --once

# Confirm SLAM is publishing map→odom
ros2 run tf2_ros tf2_echo map odom

# Check slam_toolbox lifecycle state
ros2 lifecycle get /slam_toolbox

# Inspect active topics
ros2 topic list
ros2 topic hz /scan
ros2 topic hz /imu/data
```

---

## Package dependency graph

```
robot_slam
└── slam_toolbox, rviz2

robot_simulation
├── robot_description
├── ros_gz_sim, ros_gz_bridge, ros_gz_image
├── robot_state_publisher, xacro, rviz2
└── rclpy, sensor_msgs  (for scan_relay script)

robot_description
└── xacro, robot_state_publisher, joint_state_publisher_gui, rviz2, ros_gz_sim

robot_interfaces / robot_navigation / robot_services / robot_tasks / robot_vision
└── (scaffold — no dependencies yet)
```

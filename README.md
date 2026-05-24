# UGV Rover PT — ROS 2 Workspace

A complete ROS 2 autonomy stack — from URDF model and Gazebo Harmonic simulation through online SLAM, autonomous frontier exploration, Nav2 point-to-point navigation, reactive obstacle avoidance, YOLOv8 object detection, waypoint inspection, and service-layer abstractions.

---

## Packages

| Package | Purpose |
|---|---|
| `robot_description` | Xacro URDF model (4-wheel skid-steer, pan-tilt arm, all sensors) + RViz display launch |
| `robot_simulation` | Gazebo Harmonic simulation, GZ↔ROS bridges, empty and obstacle worlds |
| `robot_slam` | Online mapping with slam_toolbox (async, lifecycle-managed) |
| `robot_explorer` | Autonomous RRT frontier exploration + heading-controller navigator |
| `robot_tasks` | Reactive explorer (`explore`), waypoint patrol (`patrol`), inspection tour (`inspection`) |
| `robot_vision` | YOLOv8 object detection on any camera topic |
| `robot_navigation` | Nav2 stack — MPPI controller, NavFn planner, AMCL or SLAM localization, optional EKF |
| `robot_interfaces` | Custom service and action definitions (`GoToPose`, `DetectObject`, `NavigateToPose`, `SearchObject`) |
| `robot_services` | Service servers: `/go_to_pose` (Nav2 wrapper) and `/detect_object` (YOLO + depth) |
| `robot_actions` | Action servers: `/robot_navigate_to_pose` (Nav2 wrapper with feedback) and `/search_object` (360° YOLO scan) |

---

## Robot Overview

The UGV Rover PT is modeled as a 4-wheel skid-steer ground vehicle (~3.5 kg, 252 × 230 × 94 mm) with a pan-tilt camera arm and deck-mounted sensor suite.

```
                       ┌─────────────┐
                       │  Deck Plate │ ← LiDAR (rear-right)
  ┌──────────────────────────────────────────┐
  │  [FL]        arm_column                  │
  │              pan_link (±90° Z)           │
  │              tilt_link (−30°…+60° Y)     │
  │              camera / depth / rail       │
  │  [RL]                              [RR]  │
  └──────────────────────────────────────────┘
  front_camera_link (126 mm stereo bar, front face)
```

### Drive

Skid-steer via the Gazebo `DiffDrive` plugin.  
All four corner wheels are driven.  
`mu1 = 1.0` (longitudinal) and `mu2 = 0.2` (lateral) on all wheels — low lateral friction allows smooth skid-steer slip during turns.

| Property | Value |
|---|---|
| Wheel radius | 41.87 mm |
| Wheel separation (track) | ~178 mm |
| Ground clearance | 25.13 mm |

### Sensors

| Sensor | Frame | ROS Topic | Rate |
|---|---|---|---|
| Deck LiDAR (GPU) | `lidar_link` | `/scan` | 15 Hz |
| Pan-tilt RGB camera | `camera_optical_frame` | `/camera/image_raw` | 30 Hz |
| Depth / IR sensor | `depth_optical_frame` | `/depth_camera/image_raw`, `/depth_camera/points` | 30 Hz |
| Front stereo bar | `front_camera_optical_frame` | `/front_camera/image_raw` | 30 Hz |
| IMU | `imu_link` | `/imu/data` | 100 Hz |

### Pan-tilt Joints

| Joint | Type | Axis | Limits |
|---|---|---|---|
| `pan_joint` | revolute | Z (yaw) | ±90° |
| `tilt_joint` | revolute | Y (pitch) | −30° … +60° |
| `*_wheel_joint` (×4) | continuous | Y | — |

---

## Prerequisites

- ROS 2 **Jazzy**
- Gazebo **Harmonic** (`gz-harmonic`)
- Python 3 with `ultralytics` (`pip install ultralytics`)

```bash
sudo apt install \
  ros-$ROS_DISTRO-ros-gz-sim \
  ros-$ROS_DISTRO-ros-gz-bridge \
  ros-$ROS_DISTRO-robot-state-publisher \
  ros-$ROS_DISTRO-slam-toolbox \
  ros-$ROS_DISTRO-navigation2 \
  ros-$ROS_DISTRO-nav2-bringup \
  ros-$ROS_DISTRO-robot-localization \
  ros-$ROS_DISTRO-rviz2 \
  ros-$ROS_DISTRO-nav2-rviz-plugins \
  ros-$ROS_DISTRO-xacro \
  ros-$ROS_DISTRO-cv-bridge \
  ros-$ROS_DISTRO-vision-msgs \
  python3-numpy
```

---

## Build

```bash
cd ~/Documents/projects/robot/p12/ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build
source install/setup.bash
```

---

## Launch Reference

### 1 — RViz model viewer (no simulation)

```bash
ros2 launch robot_description display.launch.py
```

---

### 2 — Gazebo simulation

```bash
# Empty world
ros2 launch robot_simulation gazebo.launch.py

# Obstacle arena (10×10 m, walls + pillars + boxes)
ros2 launch robot_simulation obstacle_sim.launch.py
```

| Argument | Default | Description |
|---|---|---|
| `world` | `empty.sdf` | Path to SDF world file |
| `x`, `y`, `z` | `0 0 0.15` | Spawn position (m) |
| `yaw` | `0.0` | Spawn heading (rad) |
| `rviz` | `true` | Open RViz alongside Gazebo |

---

### 3 — Reactive obstacle avoidance

Drives forward and steers away from obstacles using LiDAR sectors.  
Does not require SLAM or a map.

```bash
ros2 run robot_tasks explore
```

```bash
# Tunable parameters
ros2 run robot_tasks explore --ros-args \
  -p forward_speed:=0.3 \
  -p turn_speed:=0.8 \
  -p safety_dist:=0.8 \
  -p front_half_deg:=40.0
```

---

### 4 — Autonomous SLAM + RRT exploration

Builds a map with slam_toolbox while exploring using RRT-based frontier detection.  
Requires defining an exploration region interactively in RViz.

```bash
# Terminal 1 — simulation
ros2 launch robot_simulation gazebo.launch.py

# Terminal 2 — SLAM + exploration
ros2 launch robot_explorer explore.launch.py
```

**Setup (5 clicks in RViz — Publish Point, in order):**

```
Click 1 ──┐
Click 2   ├── Four corners of the exploration boundary
Click 3   │   (any order, e.g. clockwise)
Click 4 ──┘

Click 5 ──── RRT seed point (inside known free space on the map)
```

After click 5 the boundary polygon (green) and seed (cyan) appear in RViz and exploration begins automatically.

| Argument | Default | Description |
|---|---|---|
| `rrt_iterations` | `600` | RRT growth steps per detection cycle |
| `step_size` | `0.5` | RRT step size (m) |
| `forward_speed` | `0.20` | Navigation forward speed (m/s) |
| `safety_dist` | `0.45` | Obstacle avoidance threshold (m) |

**RViz topics to add:**

| Display | Topic | Notes |
|---|---|---|
| Map | `/map` | Grows as robot explores |
| MarkerArray | `/frontier_markers` | Orange spheres = frontier candidates |
| Marker | `/exploration_boundary` | Green polygon + cyan seed |
| Marker | `/goal_marker` | Green cylinder = current navigation goal |

Save the map when done:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/map
```

---

### 5 — Waypoint patrol

Drives a closed loop of GPS-free waypoints (defined in the odom frame) with obstacle avoidance and an RViz marker overlay.

```bash
# Default: 2×2 m square
ros2 launch robot_tasks patrol.launch.py

# Custom waypoints
ros2 launch robot_tasks patrol.launch.py \
  waypoints_x:="[0.0, 4.0, 4.0, 0.0]" \
  waypoints_y:="[0.0, 0.0, 4.0, 4.0]"
```

| Argument | Default | Description |
|---|---|---|
| `waypoints_x` / `waypoints_y` | 2×2 m square | Waypoint coordinates (m, odom frame) |
| `waypoints_yaw` | `[0,1.57,3.14,-1.57]` | Final heading at each waypoint (rad) |
| `forward_speed` | `0.2` | Drive speed (m/s) |
| `turn_speed` | `0.6` | Turn rate (rad/s) |
| `goal_tolerance` | `0.3` | Arrival radius (m) |
| `safety_dist` | `0.45` | Obstacle abort distance (m) |
| `wait_time` | `1.5` | Pause at each waypoint (s) |
| `loop` | `true` | Repeat indefinitely |

**RViz markers** published to `/patrol_markers`: blue path strip, yellow (active) / green (waiting) / blue (upcoming) cylinders.

---

### 6 — SLAM only (manual drive / map building)

Drive manually while slam_toolbox builds a map. Save the map for later use in Nav2 Mode B.

```bash
# Terminal 1 — simulation
ros2 launch robot_simulation obstacle_sim.launch.py rviz:=false

# Terminal 2 — SLAM
ros2 launch robot_slam slam.launch.py use_sim_time:=true

# Terminal 3 — teleoperation
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/cmd_vel
```

Verify SLAM is publishing:
```bash
ros2 run tf2_ros tf2_echo map odom
ros2 topic echo /map --once
```

Save the finished map:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_map
# Produces ~/my_map.yaml and ~/my_map.pgm
```

---

### 7 — Nav2 autonomous navigation

Full Nav2 stack — MPPI trajectory controller, NavFn global planner, dual costmaps, Behaviour Tree navigator, and velocity smoother. Max linear speed: **0.5 m/s**. Max angular speed: **1.0 rad/s**.

Two localization sources are supported; choose based on whether you have a pre-built map.

```
                  ┌─────────────────────────────────┐
                  │         robot_navigation         │
                  │                                  │
  /scan ──────────┤  local costmap (obstacle layer)  │
  /map  ──────────┤  global costmap (static layer)   │──► /cmd_vel
  /odom ──────────┤  MPPI controller + NavFn planner │
  /tf   ──────────┤  BT navigator (action server)    │
                  └─────────────────────────────────┘
         localization source (one of):
           A) slam_toolbox   → publishes /map + map→odom TF
           B) map_server + AMCL → loads .yaml, publishes /map + map→odom TF
```

#### Mode A — SLAM + Nav2 (map built live while navigating)

Use this when you have not yet built a map, or want to extend the map while navigating.

```bash
# Terminal 1 — simulation
ros2 launch robot_simulation obstacle_sim.launch.py

# Terminal 2 — SLAM (provides /map and map→odom TF)
ros2 launch robot_slam slam.launch.py use_sim_time:=true

# Terminal 3 — Nav2 stack
ros2 launch robot_navigation navigation.launch.py
```

**RViz setup:**

1. In the RViz toolbar click **Panels → Add New Panel → Nav2 Panel** (adds goal controls).
2. Add these displays:

| Display | Topic / Setting |
|---|---|
| Map | `/map` |
| Map | `/local_costmap/costmap` (rolling local window) |
| Map | `/global_costmap/costmap` |
| Path | `/plan` (global plan, green) |
| Path | `/local_plan` (local trajectory, red) |
| MarkerArray | `/local_costmap/voxel_layer_grid` (optional) |

3. Wait until the Nav2 Panel shows **Active** (lifecycle manager has activated all nodes).
4. Click **Navigation2 Goal** in the toolbar (or use the Nav2 Panel) and click a free cell on the map to send a goal.

Verify Nav2 is running:
```bash
ros2 action list          # should include /navigate_to_pose
ros2 topic echo /plan --once
```

---

#### Mode B — Pre-built map + AMCL + Nav2

Use this when you already have a saved map (`.yaml` + `.pgm`).  
AMCL localises the robot on the known map using particle filtering.

```bash
# Terminal 1 — simulation
ros2 launch robot_simulation obstacle_sim.launch.py

# Terminal 2 — Localization (map_server + AMCL)
ros2 launch robot_navigation localization_launch.py \
  map:=/path/to/my_map.yaml

# Terminal 3 — Nav2 stack
ros2 launch robot_navigation navigation.launch.py
```

`localization_launch.py` brings up `map_server` + `amcl` + their lifecycle manager. `navigation.launch.py` brings up the full Nav2 controller/planner/BT stack separately.

**RViz setup (same displays as Mode A, plus):**

5. Click the **2D Pose Estimate** button in the RViz toolbar.  
   Click and drag on the map at the robot's current position and heading.  
   AMCL will converge its particle cloud around that estimate (visible as a red arrow cloud).

> Without setting the initial pose AMCL has no prior and will not localise.  
> Drive the robot a short distance after setting the estimate to help the particles converge.

Verify AMCL is localising:
```bash
ros2 topic echo /amcl_pose --once
ros2 run tf2_ros tf2_echo map odom
```

---

#### Localization only (AMCL without Nav2 stack)

Useful for testing localisation independently before bringing up navigation.

```bash
ros2 launch robot_navigation localization_launch.py \
  map:=/path/to/my_map.yaml
```

Starts `map_server` + `amcl` + their lifecycle manager. Remember to set the **2D Pose Estimate** in RViz.

---

#### Optional EKF odometry fusion

Fuses wheel odometry (`/odom`) and IMU angular velocity + linear acceleration (`/imu/data`) using `robot_localization`'s EKF node. Produces a smoother odometry estimate on top of either localization mode.

```bash
# Run as a standalone node alongside the navigation stack
ros2 run robot_localization ekf_node --ros-args \
  --params-file $(ros2 pkg prefix robot_slam)/share/robot_slam/config/ekf.yaml
```

The EKF config is in `robot_slam/config/ekf.yaml`. It runs at **30 Hz** and fuses `/odom` (x, y, yaw, vx, vyaw) with `/imu/data` (yaw rate, ax). It publishes the `odom → base_footprint` TF and an `/odometry/filtered` topic.

---

#### Launch arguments (`navigation.launch.py`)

| Argument | Default | Description |
|---|---|---|
| `use_sim_time` | `true` | Use Gazebo `/clock` |

#### Launch arguments (`localization_launch.py`)

| Argument | Default | Description |
|---|---|---|
| `map` | *(required)* | Full path to map `.yaml` file |
| `use_sim_time` | `true` | Use Gazebo `/clock` |

---

#### Nav2 node graph

| Node | Package | Role |
|---|---|---|
| `controller_server` | `nav2_controller` | MPPI local trajectory controller → `/cmd_vel` |
| `planner_server` | `nav2_planner` | NavFn A* global path planner |
| `smoother_server` | `nav2_smoother` | Path smoother |
| `behavior_server` | `nav2_behaviors` | Recovery behaviours (spin, backup, wait) |
| `bt_navigator` | `nav2_bt_navigator` | Behaviour Tree action server (`/navigate_to_pose`) |
| `waypoint_follower` | `nav2_waypoint_follower` | Sequential waypoint action server |
| `velocity_smoother` | `nav2_velocity_smoother` | Rate-limits `/cmd_vel` acceleration |
| `lifecycle_manager_navigation` | `nav2_lifecycle_manager` | Brings all above nodes through configure → active |
| `map_server` *(Mode B)* | `nav2_map_server` | Loads `.yaml` map, publishes `/map` |
| `amcl` *(Mode B)* | `nav2_amcl` | Particle-filter localisation, publishes `map→odom` TF |
| `lifecycle_manager_localization` *(Mode B)* | `nav2_lifecycle_manager` | Manages `map_server` + `amcl` |
| `ekf_filter_node` *(optional)* | `robot_localization` | Fuses `/odom` + `/imu/data` |

---

### 8 — YOLO object detection

Subscribes to a camera topic and publishes annotated images and detections.  
Model is downloaded automatically from Ultralytics on first run.

```bash
ros2 launch robot_vision vision.launch.py
```

```bash
# GPU + larger model + different camera
ros2 launch robot_vision vision.launch.py \
  model:=yolov8s.pt \
  device:=cuda:0 \
  image_topic:=/camera/image_raw \
  confidence:=0.4
```

| Argument | Default | Description |
|---|---|---|
| `model` | `yolov8n.pt` | Ultralytics model file or name |
| `confidence` | `0.5` | Detection confidence threshold |
| `device` | `cpu` | `cpu`, `cuda:0`, or `mps` |
| `image_topic` | `/front_camera/image_raw` | Input image topic |

**Output topics:**

| Topic | Type | Content |
|---|---|---|
| `/vision/detections` | `vision_msgs/Detection2DArray` | Bounding boxes, class names, confidence |
| `/vision/image_detected` | `sensor_msgs/Image` | Annotated frame with drawn boxes |

---

### 9 — Inspection tour

Drives the robot to each waypoint, performs a 360° rotation scan to detect objects with YOLO, then prints a summary of all unique sightings. Uses LiDAR for obstacle avoidance between waypoints.

```bash
# Terminal 1 — simulation + YOLO
ros2 launch robot_simulation obstacle_sim.launch.py
ros2 launch robot_vision vision.launch.py

# Terminal 2 — inspection
ros2 launch robot_tasks inspection.launch.py
```

```bash
# Custom waypoints (e.g. southern half of obstacle arena)
ros2 launch robot_tasks inspection.launch.py \
  waypoints_x:="[0.0, 3.0, 0.0, -3.0]" \
  waypoints_y:="[0.0, -2.5, -4.0, -2.5]"
```

**State machine:**

| State | Behaviour |
|---|---|
| `ALIGNING` | Rotates in place toward the next waypoint |
| `DRIVING` | Drives forward with proportional speed and soft heading correction |
| `AVOIDING` | Obstacle within `safety_dist` — turns toward the more open side |
| `SCANNING` | Rotates in place for `scan_duration` seconds (~one full revolution) |
| `DONE` | All waypoints visited; final sighting summary printed to the log |

**Detection during scan:**  
Each YOLO bounding box is projected into the world frame by taking the horizontal offset of the bbox centre relative to the camera FOV, looking up the nearest LiDAR range in that direction, and computing `(rx + range·cos(ryaw + bearing), ry + range·sin(ryaw + bearing))`. Sightings of the same class within `merge_dist` are merged into a running average.

| Argument | Default | Description |
|---|---|---|
| `waypoints_x` / `waypoints_y` | 2×2 m square | Inspection waypoints (m, odom frame) |
| `forward_speed` | `0.2` | Drive speed (m/s) |
| `turn_speed` | `0.6` | Turn rate while navigating (rad/s) |
| `scan_turn_speed` | `0.8` | Turn rate during 360° scan (rad/s) |
| `goal_tolerance` | `0.3` | Arrival radius (m) |
| `safety_dist` | `0.45` | Obstacle abort distance (m) |
| `scan_duration` | `8.0` | Seconds to spin at each waypoint (~360° at 0.8 rad/s) |
| `camera_fov_deg` | `90.0` | Horizontal FOV of front camera (degrees) |
| `img_width` | `640.0` | Detection image width (pixels) |
| `merge_dist` | `0.5` | Radius to merge duplicate sightings (m) |
| `loop` | `false` | Repeat the inspection tour indefinitely |

**RViz markers** published to `/inspection/markers`:

| Marker | Colour | Meaning |
|---|---|---|
| Waypoint cylinder | Blue | Pending |
| Waypoint cylinder | Yellow | Robot heading here |
| Waypoint cylinder | Orange | Currently scanning |
| Sphere + label | Red | Detected object (world position estimate) |

---

### 10 — Service servers

Thin service wrappers that other nodes and scripts can call programmatically.

```bash
ros2 launch robot_services service_server.launch.py
```

| Argument | Default | Description |
|---|---|---|
| `use_sim_time` | `true` | Use Gazebo `/clock` |

**`/go_to_pose`** — `robot_interfaces/srv/GoToPose`  
Sends a goal to Nav2's `navigate_to_pose` action and blocks until it succeeds or fails.  
Requires Nav2 to be running.

```bash
ros2 service call /go_to_pose robot_interfaces/srv/GoToPose \
  "{x: 2.0, y: 1.5, yaw: 1.57}"
```

```
Request:   float64 x, float64 y, float64 yaw
Response:  bool success, string message
```

**`/detect_object`** — `robot_interfaces/srv/DetectObject`  
Returns the highest-confidence YOLO detection of a named class, with its 3-D pose estimated from the depth camera and projected to the map frame.  
Requires `robot_vision` YOLO node and a depth camera bridge.

```bash
ros2 service call /detect_object robot_interfaces/srv/DetectObject \
  "{target_class: 'person'}"
```

```
Request:   string target_class
Response:  bool found, geometry_msgs/PoseStamped pose, float32 confidence
```

---

### 11 — Action servers

Long-running action wrappers that publish incremental feedback while executing.

```bash
ros2 launch robot_actions action_server.launch.py
```

| Argument | Default | Description |
|---|---|---|
| `use_sim_time` | `true` | Use `/clock` from simulation instead of wall time |

**`/robot_navigate_to_pose`** — `robot_interfaces/action/NavigateToPose`  
Forwards a goal to Nav2's `navigate_to_pose` action and streams `distance_remaining` + `current_state` feedback every 500 ms. Supports cancellation. Requires Nav2 to be running.

```
Goal:     float64 x, float64 y, float64 yaw
Feedback: float32 distance_remaining, string current_state
            current_state: "Following path" | "Arriving" | "Recovering (N)"
Result:   bool success, string message
```

```bash
ros2 action send_goal /robot_navigate_to_pose \
  robot_interfaces/action/NavigateToPose \
  "{x: 2.0, y: 1.5, yaw: 1.57}" --feedback
```

---

**`/search_object`** — `robot_interfaces/action/SearchObject`  
Rotates the robot 360° at 0.3 rad/s while polling YOLO detections. Stops as soon as the target class is found above the confidence threshold, computes its 3-D world position from the depth camera, saves a JPEG snapshot to `/tmp/`, and succeeds. Aborts if the full rotation completes without a match. Requires `robot_vision` (`vision.launch.py`) to be running.

```
Goal:     string target_class, float32 min_confidence
Feedback: string search_state, float32 progress_percent, int32 detections_count
            search_state: "Scanning" | "Detected (N)" | "Found" | "Not found"
Result:   bool found, float64 x, float64 y, float64 z, string image_path
```

```bash
ros2 action send_goal /search_object \
  robot_interfaces/action/SearchObject \
  "{target_class: 'person', min_confidence: 0.5}" --feedback
```

Topics subscribed during execution:

| Topic | Use |
|---|---|
| `/vision/detections` | YOLO bounding boxes (class + confidence) |
| `/camera/image_raw` | RGB frame saved on detection |
| `/depth_camera/image_raw` | Depth (32FC1) for 3-D position |
| `/depth_camera/camera_info` | Intrinsics for back-projection |

---

## Custom Interfaces (`robot_interfaces`)

### Services

| Service | File | Description |
|---|---|---|
| `GoToPose` | `srv/GoToPose.srv` | Navigate to (x, y, yaw) via Nav2 |
| `DetectObject` | `srv/DetectObject.srv` | Find a named YOLO class; return 3-D pose |

### Actions

| Action | File | Description |
|---|---|---|
| `NavigateToPose` | `action/NavigateToPose.action` | Navigate to (x, y, yaw) via Nav2 with distance feedback |
| `SearchObject` | `action/SearchObject.action` | 360° YOLO scan; returns 3-D pose + image path on match |

---

## Full Topic Reference

### Simulation / Hardware

| Topic | Type | Source |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | ROS → robot (DiffDrive input) |
| `/odom` | `nav_msgs/Odometry` | DiffDrive plugin |
| `/tf` | `tf2_msgs/TFMessage` | Gazebo + slam_toolbox / AMCL |
| `/joint_states` | `sensor_msgs/JointState` | Gazebo |
| `/clock` | `rosgraph_msgs/Clock` | Gazebo |

### Sensors

| Topic | Type | Source |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | LiDAR (via scan_relay) |
| `/imu/data` | `sensor_msgs/Imu` | IMU |
| `/camera/image_raw` | `sensor_msgs/Image` | Pan-tilt RGB camera |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Pan-tilt RGB camera |
| `/front_camera/image_raw` | `sensor_msgs/Image` | Front stereo camera |
| `/front_camera/camera_info` | `sensor_msgs/CameraInfo` | Front stereo camera |
| `/depth_camera/image_raw` | `sensor_msgs/Image` | Depth sensor (32FC1) |
| `/depth_camera/camera_info` | `sensor_msgs/CameraInfo` | Depth sensor |
| `/depth_camera/points` | `sensor_msgs/PointCloud2` | Depth sensor |

### Mapping & Localisation

| Topic | Type | Source |
|---|---|---|
| `/map` | `nav_msgs/OccupancyGrid` | slam_toolbox or map_server |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | amcl |
| `/particle_cloud` | `geometry_msgs/PoseArray` | amcl |

### Navigation (Nav2)

| Topic | Type | Source / Sink |
|---|---|---|
| `/goal_pose` | `geometry_msgs/PoseStamped` | RViz → bt_navigator |
| `/plan` | `nav_msgs/Path` | planner_server (global path) |
| `/local_plan` | `nav_msgs/Path` | controller_server (MPPI rollout) |
| `/cmd_vel_nav` | `geometry_msgs/Twist` | controller_server → velocity_smoother |
| `/local_costmap/costmap` | `nav_msgs/OccupancyGrid` | local_costmap |
| `/local_costmap/costmap_raw` | `nav2_msgs/Costmap` | local_costmap |
| `/local_costmap/published_footprint` | `geometry_msgs/PolygonStamped` | local_costmap |
| `/global_costmap/costmap` | `nav_msgs/OccupancyGrid` | global_costmap |
| `/global_costmap/costmap_raw` | `nav2_msgs/Costmap` | global_costmap |
| `/waypoints` | `nav_msgs/Path` | waypoint_follower input |

### Navigation Actions (Nav2)

| Action | Type | Server |
|---|---|---|
| `/navigate_to_pose` | `nav2_msgs/NavigateToPose` | bt_navigator |
| `/navigate_through_poses` | `nav2_msgs/NavigateThroughPoses` | bt_navigator |
| `/follow_waypoints` | `nav2_msgs/FollowWaypoints` | waypoint_follower |
| `/compute_path_to_pose` | `nav2_msgs/ComputePathToPose` | planner_server |
| `/follow_path` | `nav2_msgs/FollowPath` | controller_server |

### Robot Actions

| Action | Type | Server |
|---|---|---|
| `/robot_navigate_to_pose` | `robot_interfaces/action/NavigateToPose` | navigate_to_pose_action_server |
| `/search_object` | `robot_interfaces/action/SearchObject` | search_object_action_server |

### Autonomous Exploration

| Topic | Type | Source |
|---|---|---|
| `/detected_points` | `geometry_msgs/PointStamped` | frontier_detector |
| `/frontier_markers` | `visualization_msgs/MarkerArray` | frontier_detector |
| `/exploration_boundary` | `visualization_msgs/Marker` | frontier_detector |
| `/goal_marker` | `visualization_msgs/Marker` | explorer |

### Tasks

| Topic | Type | Source |
|---|---|---|
| `/patrol_markers` | `visualization_msgs/MarkerArray` | patrol node |
| `/inspection/markers` | `visualization_msgs/MarkerArray` | inspection node |

### Vision

| Topic | Type | Source |
|---|---|---|
| `/vision/detections` | `vision_msgs/Detection2DArray` | yolo_node |
| `/vision/image_detected` | `sensor_msgs/Image` | yolo_node |

### Services

| Service | Type | Server |
|---|---|---|
| `/go_to_pose` | `robot_interfaces/srv/GoToPose` | go_to_pose_server |
| `/detect_object` | `robot_interfaces/srv/DetectObject` | detect_object_server |

---

## TF Tree

```
map                           ← slam_toolbox
└── odom
    └── base_footprint
        └── base_link
            ├── front_left_wheel / front_right_wheel
            ├── rear_left_wheel  / rear_right_wheel
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

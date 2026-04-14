Back to [Main Project Home](https://github.com/slgrobotics/articubot_one/wiki)

## ROS2 Geofence Manager

A ROS2 package for **polygon-based geofencing**, designed for outdoor patrolling robots using GPS, Nav2, and Behavior Trees.

This node provides:
- Real-time geofence status
- Distance and nearest-boundary geometry
- Visualization in RViz
- Helper services for navigation behaviors (e.g., bounce target)

It supports:
- Simple YAML-defined polygons (for debugging or when X,Y local coordinates are available)
- QGroundControl (`*.plan`) geofences - defined in (lat,lon) terms
- Proper *WGS84* → *ROS `map` frame* conversion using ROS services

<img width="2033" height="783" alt="Screenshot from 2026-04-14 15-59-44" src="https://github.com/user-attachments/assets/e88da6c5-1119-4f84-943a-39b0a404e1b8" />

### Overview

The `geofence_manager` node acts as a **geometry and safety layer** between localization and behavior.

It continuously evaluates the robot pose relative to a polygon (geofence) and publishes:

- Whether the robot is inside/outside
- Distance to boundary
- Nearest boundary point
- Stabilized state (with hysteresis)

It also provides services that higher-level logic (Behavior Trees, patrol manager, recovery logic) can use.

### Key Features

- Polygon-based geofence (convex or concave)
- Hysteresis-based state stabilization (INSIDE / NEAR / OUTSIDE)
- Nearest boundary point computation
- Distance-to-boundary metric
- Inward-normal visualization
- Bounce target computation (for wandering / recovery)
- QGC `.plan` support with **ROS-consistent coordinate conversion**

### Example Usage

#### Build

```
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src
git clone https://github.com/slgrobotics/ros2_geofence_manager.git
cd ~/robot_ws
colcon build
```

#### Test (stand-alone, GUI):
```
cd ~/robot_ws/src/ros2_geofence_manager/test
./test_bounce.py --file ../plans/geofence_polygon.yaml --x 1.2 --y 3.2 --angle-deg 30 --angle-jitter 10
  or
./test_bounce.py --file ../plans/geofence_qgroundcontrol.plan --x 33.19983710 --y -86.29979086 --angle-deg 30 --angle-jitter 10
```

<img width="998" height="1037" alt="Screenshot from 2026-04-14 16-01-42" src="https://github.com/user-attachments/assets/3bb79292-1dc0-4a61-ae9f-486f496ed484" />

#### Node Launch

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
cd ~/robot_ws
colcon build
source ~/robot_ws/install/setup.bash
ros2 launch geofence_manager geofence.launch.py
```

With simulation time (and Dragger robot, see [this section](https://github.com/slgrobotics/robots_bringup/blob/main/Docs/ROS-Jazzy/README.md#bringing-up-robot-simulation-in-gazebo)):

```bash
ros2 launch geofence_manager geofence.launch.py use_sim_time:=true
```

### Architecture

#### Core Node

```
geofence_manager_node
```

Loads:
- QGroundControl *.plan* file or simlified *.yaml* file with geofence polygon definition.

Consumes:
- Robot pose (e.g. `/odometry/global`)

Publishes:
- Geofence status and geometry

Provides:
- Geometry-based services

#### Geometry Pipeline

```
Robot Pose (map frame)
        ↓
BoundaryContext (geometry core)
        ↓
State classification + metrics
        ↓
Topics + Services
```

### Coordinate Frames (Important)

This package assumes **all runtime geometry is in a single Cartesian frame** (typically `map`).

#### QGC Integration (Correct Way)

QGC *.plan* files define geofences in:

```
(latitude, longitude)  (WGS84)
```

These are converted using ROS services:

- `/fromLL` → converts `(lat, lon)` → `(x, y)` in `map`
- `/toLL` → converts `(x, y)` → `(lat, lon)`

This is done via `navsat_transform_node`, which ensures GPS coordinates are consistent with the robot’s world frame (usually `map`).

### Supported Geofence Formats

#### 1. YAML (local Cartesian)

```yaml
geofence:
  name: home_area
  frame_id: map
  points:
    - {x: -3.5, y: -3.5}
    - {x: -3.5, y: 3.5}
    - {x: 3.5, y: 3.5}
    - {x: 3.5, y: -3.5}
```

#### 2. QGroundControl `.plan`

Extracted from:

```
geoFence.polygons[*].polygon = [
  [lat, lon],
  ...
]
```

Converted internally via:

```
(lat, lon) --> /fromLL ROS service --> (x, y) in "map" reference frame
```

### Topics

#### Output

- `/geofence/status`  
  → `GeofenceStatus`

- `/geofence/is_inside`  
  → `std_msgs/Bool`

- `/geofence/nearest_boundary_point`  
  → `geometry_msgs/PointStamped`

- `/geofence/distance_to_boundary`  
  → `std_msgs/Float32`

- `/geofence/polygon`  
  → `geometry_msgs/PolygonStamped` (latched)

- `/geofence/markers`  
  → `visualization_msgs/MarkerArray`

### Services

- `/geofence/is_pose_allowed`  
  → Check if a pose is inside the geofence

- `/geofence/compute_bounce_target`  
  → Compute a navigation target based on boundary interaction

### Visualization

RViz markers include:

- Boundary polygon (line strip)
- Vertices
- **Dynamic inward normal** (nearest boundary → inside direction)

### State Model

The node classifies robot position into:

- `INSIDE`
- `NEAR_BOUNDARY`
- `OUTSIDE`
- `UNKNOWN`

Transitions are stabilized to compensate for robot position jitter using:
- Hysteresis 
- Debounce counters

### Bounce Target Logic

The node provides a **geometry-driven recovery/wandering primitive**:

- Computes nearest boundary
- Computes inward normal
- Applies angle + jitter
- Returns a valid interior target

Used for:
- Patrol
- Wandering
- Boundary recovery

### QGC Integration Details

The ROS-aware loader: `helpers/qgc_plan_loader_ros.py`

Performs:

1. Parse `.plan`
2. Extract polygon (WGS84)
3. Call `/fromLL` for each vertex
4. Build `map`-frame polygon
5. Call `/toLL(0,0)` to record map origin in WGS84

This ensures:

- Geofence aligns exactly with `/odometry/global`
- No drift or mismatch between GPS and geometry

### Dependencies

- ROS2 (Jazzy+ recommended)
- `robot_localization`
- `navsat_transform_node`
- `geometry_msgs`
- `visualization_msgs`
- `geographic_msgs`

### Limitations

- Assumes **single reference frame** - e.g. "map" (no TF transforms inside node)
- Requires:
  - valid GPS (or sim)
  - working `navsat_transform_node`
- Accuracy depends on:
  - GPS quality
  - IMU heading alignment and other positioning factors

### Design Philosophy

- Keep geometry deterministic and testable
- Keep ROS interaction minimal and explicit
- Separate:
  - parsing
  - coordinate transforms
  - geometry logic
  - behavior

### Future Work

- Multiple geofence zones
- Inclusion/exclusion regions
- Dynamic geofence updates
- Integration with Nav2 costmaps
- Patrolling robot manager package integration

### Summary

This package provides a **clean, ROS-native way to enforce geofences outdoors**:

- Uses QGC for intuitive planning
- Uses ROS services for correct coordinate alignment
- Provides reusable geometry primitives for autonomy

It is designed to sit between:
- localization (GPS / EKF)
- and behavior (Nav2 / BT)

and make geofencing **reliable, observable, and extensible**.

---------------------

Back to [Main Project Home](https://github.com/slgrobotics/articubot_one/wiki)

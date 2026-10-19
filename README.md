# My Drone Control — Map Processing and Mission Path Planning

A ROS 2 package for 3D PCD map processing, voxel grid downsampling, 3D obstacle inflation, spatial occupancy querying, mission CSV waypoint parsing, 3D A* path planning, and line-of-sight path post-processing for autonomous drone navigation in hangar environments.

## Contents

- [A1.1 — Map processing and path planning](#a1-1-map-processing)
- [A1.2 — Mission execution, indoor flight](#a1-2-mission-execution)
- [Prerequisites and build](#prerequisites-and-build)
- [How to run](#how-to-run)
- [ROS 2 interface](#ros-2-interface)
- [Mission pipeline](#mission-pipeline)
- [Implementation overview](#implementation-overview)
- [Launch file](#launch-file)

---

<a id="a1-1-map-processing"></a>
## A1.1 — Map processing and path planning

### 1. Map loading (1.5 pts)

* **Map Source**: FEI LRS Point Cloud (`maps/map.pcd`).

* **Loader**: The map is loaded using the Point Cloud Library (PCL)
  through `pcl::io::loadPCDFile`. The raw PCD file is stored as a
  `pcl::PointCloud<pcl::PointXYZ>`.

* **Downsampling Strategy**: The raw point cloud is downsampled using a 3D Voxel Grid filter (`pcl::VoxelGrid`) with a configurable leaf size of **0.25 m by default**.

* **3D Spatial Representation**: Each downsampled point is converted
  into a discrete 3D voxel represented by `Index3D`, containing
  integer `x`, `y`, and `z` indices. Occupied voxels are stored in an
  `unordered_set<Index3D, Index3DHash>`, providing average-case
  $O(1)$ occupancy lookup.


* **Coordinate Conversion**:

  * World coordinates are converted to voxel indices using `pointToIndex()`.
  * Voxel indices are converted back to voxel-center coordinates using `indexToPoint()`.

![3D voxel representation of the FEI LRS map](readme_data/voxel_grid.png)

*Figure 1: Raw FEI LRS point cloud and its 3D voxel-grid representation
visualized together in RViz. The map is represented directly in X, Y
and Z rather than using stacked 2D layers.*

#### Measured Map Metrics

* **World Bounding Box**:
  - $X \in [-0.30, 17.95]$ m
  - $Y \in [-1.15, 13.20]$ m
  - $Z \in [0.00, 5.95]$ m

* **Spatial Resolution**: $0.25\text{ m}$ voxel leaf size.

* **Occupied Voxel Count**: $16482$ Reported from
  `occupied_grid_.size()` after voxelization.

* **Spatial Occupancy Query**: A world-space coordinate is converted
  to an `Index3D` using `pointToIndex()` and checked against the
  occupancy representation. The underlying hash lookup provides
  average-case $O(1)$ complexity.

---

### 2. Obstacle inflation (0.5 pts)

#### Safety Radius

The planner uses a configurable obstacle inflation radius:

```text
safety_radius = 0.60 m
```
The value of $0.60$ m is based on the estimated drone body radius,
position-controller tolerance, and an additional safety margin:

$$ r_{\text{safety}} = r_{\text{drone}} + r_{\text{controller}} + r_{\text{margin}} $$

Using the project values:

$$ 0.33 + 0.15 + 0.12 = 0.60\text{ m} $$

This provides clearance for the physical drone body as well as
position-control error and an additional safety margin.

The implementation converts the physical radius into a number of voxels:

$$
r_{\text{voxels}}
=
\left\lceil
\frac{r_{\text{safety}}}{r_{\text{voxel}}}
\right\rceil
$$

With the default values of $r_{\text{safety}} = 0.60$ m and
$r_{\text{voxel}} = 0.25$ m, the resulting inflation radius is
$3$ voxels.

For every occupied voxel, neighboring voxels are examined in all three
dimensions. A voxel is added to the inflated obstacle representation if:

$$
dx^2 + dy^2 + dz^2
\leq
r_{\text{voxels}}^2
$$

This produces a spherical 3D inflation around each occupied voxel.

![3D inflated voxel grid](readme_data/voxel_inflated.png)

*Figure 2: 3D inflated voxel representation visualized in RViz. The
obstacles are expanded in X, Y and Z according to the configured
0.60 m safety radius.*

#### Steel Racks / Shelving Handling

Steel shelving and other obstacles represented by the point cloud are converted
into occupied voxels and then expanded using the 3D spherical inflation process.

The inflation can fill small gaps between nearby occupied voxels, producing a
more conservative obstacle representation. The A* planner then treats the
resulting inflated voxels as occupied.

---

### 3. Advanced planning algorithm — A* (2.0 pts)

* **Algorithm**: 3D A* grid search using a **26-connected neighborhood**. Each voxel can have up to 26 neighboring voxels.

* **Search Space**: The planner operates directly on the inflated voxel grid.

* **Heuristic**: 3D Euclidean distance:

$$
h(n)
=
\sqrt{
\Delta x^2 +
\Delta y^2 +
\Delta z^2
}
$$

The Euclidean heuristic is admissible because it represents the
shortest possible geometric distance between the current voxel and
the goal. Obstacles can only increase the actual path cost, so the
heuristic never overestimates the optimal remaining cost.

* **A* Cost Function**:

$$
f(n) = g(n) + h(n)
$$

where:

* $g(n)$ is the accumulated cost from the start voxel to node $n$.

* $h(n)$ is the Euclidean heuristic estimate from node $n$ to the goal.

* $f(n)$ is the estimated total path cost through node $n$.

* **Movement Cost**:

$$
c(n,n')
=
\sqrt{
\Delta x^2 +
\Delta y^2 +
\Delta z^2
}
$$

This produces:

* Axis-aligned movement: $1$

* 2D diagonal movement: $\sqrt{2}$

* 3D diagonal movement: $\sqrt{3}$

* **Open Set**: A priority queue stores nodes ordered by their lowest $f$ score.

* **Closed Set**: An `unordered_set` stores already expanded voxels.

* **Cost Tracking**: `g_score` stores the best known cost from the start to each voxel.

* **Path Reconstruction**: `came_from` stores the predecessor of each voxel. When the goal is reached, the path is reconstructed backwards and then reversed.

* **Collision Checking**: Neighboring voxels contained in the inflated occupancy grid are rejected.

* **Sequential Waypoint Planning**: The mission CSV is interpreted as an ordered list of waypoints. A separate A* search is performed between every pair of consecutive waypoints.

* **Unreachable Goals**: If the start or goal voxel is occupied, or the A* search cannot reach the goal, an empty path is returned and mission planning stops.

* **Performance Measurement**: The complete mission planning process is timed using `std::chrono::high_resolution_clock` and the elapsed time is reported in milliseconds.

---

### Mission CSV Processing

Mission waypoints are loaded from a CSV file.

Expected format:

```text
x,y,z,precision,task
```

For example:

```text
0.0,0.0,1.0,0.1,takeoff
4.0,2.0,2.0,0.1,inspection
8.0,4.0,2.5,0.1,landing
```

The parser:

* skips empty lines,
* skips lines beginning with `#`,
* skips the header row,
* converts `x`, `y`, and `z` to `double`,
* stores `precision` and `task` as strings,
* ignores malformed numeric rows.

Waypoint coordinates (`x`, `y`, `z`) determine the A* search. The `precision` and `task` fields do not affect route search; the planner preserves them on the corresponding mission waypoint when it publishes the executable mission.

---

### 4. Path post-processing (1.0 pt)

The raw A* path consists of neighboring voxels and may contain many unnecessary intermediate points.

The planner therefore performs a second processing stage using `simplifyPath()`.

* **Line-of-Sight Shortcutting**: The simplifier attempts to connect the current path point directly to the furthest future point that can be reached without intersecting an occupied inflated voxel.

* **3D Segment Checking**: `isLineOfSightFree()` samples positions along the 3D segment between two voxel indices.

* **Voxel Conversion**: The sampled coordinates are rounded to the nearest voxel index.

* **Collision Check**: Every sampled voxel is checked against the inflated occupancy grid.

* **Greedy Simplification**: For every current point, the algorithm searches for the furthest future point with a clear line of sight and adds that point to the simplified path.

This is a sampled 3D line-of-sight check rather than a classical DDA voxel traversal algorithm.

#### Waypoint Reduction Evidence

| Route Test Case          | Mission Source                   | Raw A* Points | Simplified Points | Point Reduction | Execution Time | Safety Check |
| ------------------------ | -------------------------------- | ------------: | ----------------: | --------------: | -------------: | ------------ |
| **Exam Mission 1**       | `exam_example.csv` (7 waypoints) |           172 |                24 |       **86.0%** |    ~2563.44 ms | PASSED       |

The reported execution time measures the complete sequential mission planning process, including the A* searches and path simplification.

![Final post-processed 3D path](readme_data/final_path.png)

*Figure 4: Final post-processed 3D mission path visualized in RViz. The
displayed path is the result of 3D A* planning followed by line-of-sight
shortcutting, and is shown together with the inflated voxel map and
mission waypoints.*

[Back to Contents](#contents)

<a id="a1-2-mission-execution"></a>
## A1.2 — Mission execution, indoor flight

This section describes the assignment to execute the A1.1 route in the hangar
world. The mission CSV contains **goal waypoints**, not the complete flight
path: the planner finds a collision-free route between consecutive rows, and
the controller performs each task at the original waypoint.

Mission files use the format `x,y,z,precision,task`. The assignment requires
loading a selected file at runtime, performing the MAVROS connection, `GUIDED`
mode, and arm handshake with response checks, then taking off, navigating,
performing each task in order, and landing/disarming. State transitions should
be logged so failures identify the stage where execution stopped.

The assignment distinguishes transit points from mission waypoints. Simplified
path points between CSV rows are pass-through targets; they must not make the
drone stop at every point. A `soft` waypoint allows transit within a looser
position tolerance, while a `hard` waypoint requires the drone to settle before
continuing. The selected tolerances and stability rule should be justified and
verified during flight.

Tasks include `takeoff`, `land`, `landtakeoff` (land, then re-arm and re-enter
`GUIDED` before continuing), and `yaw<degrees>`. Document the yaw frame and
direction convention, hold the requested heading at a yaw waypoint, and face
the direction of travel between waypoints. Validate the complete mission in
simulation before using hardware.

### A1.2 acceptance checklist

- The mission runs end-to-end from a runtime-selected mission file and lands safely.
- Tasks execute at the original mission waypoints, not at every planner point.
- `hard` and `soft` behavior is visibly different; hard arrival includes settling.
- The vehicle faces along each segment and holds commanded yaw tasks.
- The route avoids inflated obstacles, including the shelving racks.

[Back to Contents](#contents)

---

<a id="prerequisites-and-build"></a>
## 2. Prerequisites & Build Instructions

Ensure that ROS 2, PCL, and the package dependencies are installed.

Build the package with:

```bash
cd /home/user/ros2_ws

colcon build --packages-select my_drone_control

source install/setup.bash
```

[Back to Contents](#contents)

---

<a id="how-to-run"></a>
## 3. How to Run

### One-command startup

Build and source the workspace, then run the unified launch file:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select my_drone_control
source install/setup.bash
ros2 launch my_drone_control planner.launch.py
```

The launch prompts `Open RViz? [y/N]`, then starts `map_planner_node`, optional
RViz, Gazebo, ArduPilot SITL, MAVROS, and finally `takeoff_drone`. It uses the
bundled `maps/map.pcd` and `missions/exam_example.csv`. Set `LRS_REPO_DIR` if
the simulator repository is not at `~/LRS-URK`:

```bash
LRS_REPO_DIR=/path/to/LRS-URK ros2 launch my_drone_control planner.launch.py
```

Press **Ctrl+C** in the launch terminal to stop the managed processes. This
launch starts the flight controller, which can arm and execute the mission
once MAVROS connects. Use the SITL simulation and verify the selected mission
before running it.

### Manual startup

Use separate terminals to inspect or control processes individually. In each
ROS terminal, first source ROS and the workspace:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

Start the following processes in order:

1. **Gazebo**, from the LRS-URK repository:
  ```bash
  cd ~/LRS-URK
  ./scripts/run_gazebo.sh
  ```
2. **ArduPilot SITL**, in another terminal:
  ```bash
  cd ~/LRS-URK
  ./scripts/run_sitl.sh
  ```
3. **MAVROS**, in another sourced terminal:
  ```bash
  cd ~/LRS-URK
  ./scripts/run_mavros.sh
  ```
4. **Map planner**, in a sourced ROS terminal:
  ```bash
  ros2 run my_drone_control map_planner_node
  ```
5. **Takeoff and mission controller**, in another sourced ROS terminal:
  ```bash
  ros2 run my_drone_control takeoff_drone
  ```
6. **RViz, optionally**, in another sourced ROS terminal:
  ```bash
  rviz2 -d $(ros2 pkg prefix my_drone_control)/share/my_drone_control/config/planner_view.rviz
  ```

Stop each manual process with Ctrl+C in its terminal. To use another mission
CSV, pass it to the planner:

```bash
ros2 run my_drone_control map_planner_node --ros-args -p mission_path:=/path/to/mission.csv
```

The planner also accepts `map_path`, `voxel_size`, and `safety_radius` overrides.

[Back to Contents](#contents)

---

<a id="ros-2-interface"></a>
## 4. ROS 2 Interface

### Published Topics

All publishers use a QoS profile with depth `1` and `transient_local` durability.

| Topic | Type | Description |
| --- | --- | --- |
| `/map_raw` | `sensor_msgs/msg/PointCloud2` | Original PCD point cloud. |
| `/voxel_grid` | `sensor_msgs/msg/PointCloud2` | Downsampled point cloud produced by the PCL VoxelGrid filter. |
| `/voxel_inflated` | `sensor_msgs/msg/PointCloud2` | Inflated 3D occupancy representation. |
| `/planned_path` | `nav_msgs/msg/Path` | Simplified route for RViz visualization. |
| `/planned_commands` | `my_drone_control/msg/PlannedMission` | Executable route with mission precision and task metadata for `takeoff_drone`. |

All map and route positions use the `map` frame. The mission topic uses
transient-local durability so the takeoff node can receive a mission published
before it starts.

### Parameters

| Parameter       | Type     |                                                             Default | Description                          |
| --------------- | -------- | ------------------------------------------------------------------: | ------------------------------------ |
| `map_path`      | `string` |              `/home/user/ros2_ws/src/my_drone_control/maps/map.pcd` | Path to the PCD map file.            |
| `mission_path`  | `string` | `/home/user/ros2_ws/src/my_drone_control/missions/exam_example.csv` | Path to the mission CSV file.        |
| `voxel_size`    | `double` |                                                              `0.25` | Voxel-grid resolution in meters.     |
| `safety_radius` | `double` |                                                              `0.60` | Obstacle inflation radius in meters. |

> **Note:** The default paths are absolute paths from the development workspace and may need to be changed when the package is used in another workspace or on another computer.

[Back to Contents](#contents)

---

<a id="mission-pipeline"></a>
## 5. Mission Pipeline

```text
PCD map -> voxel grid -> inflated occupancy map
          |
Mission CSV -> sequential A* -> simplified route
          |
         +------------+-------------+
         |                          |
         v                          v
   /planned_path (Path)       /planned_commands
     RViz                 (PlannedMission)
                |
                v
            takeoff_drone
```

[Back to Contents](#contents)

---

<a id="implementation-overview"></a>
## 6. Implementation Overview

The planner loads the PCD and mission CSV, builds an inflated voxel map, runs
3D A* between consecutive mission waypoints, and simplifies each route segment.
It publishes the route for visualization on `/planned_path` and the executable
mission, including waypoint task metadata, on `/planned_commands`.

[Back to Contents](#contents)

---

<a id="launch-file"></a>
## 7. Launch File

`launch/planner.launch.py` is the interactive full-stack launch described in
[How to Run](#how-to-run). It asks whether to start RViz, launches the planner,
then starts the simulator and mission controller. Use the manual startup
instructions when you want each process in its own terminal.

[Back to Contents](#contents)

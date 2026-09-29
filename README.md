# My Drone Control — Map Processing and Mission Path Planning

A ROS 2 package for 3D PCD map loading, voxel grid downsampling, 3D obstacle inflation, spatial occupancy querying, mission CSV waypoint parsing, 3D A* path planning, and path simplification for autonomous drone navigation in hangar environments.

---

## 1. Documentation & Technical Specifications (Sub-assignment A1.1–A1.3)

### Map Processing & Representation (1.5 pts)

* **Map Source**: FEI LRS Point Cloud (`maps/map.pcd`).
* **Loader**: Implemented using Point Cloud Library (`pcl::io::loadPCDFile`). Raw PCD points are loaded into a `pcl::PointCloud<pcl::PointXYZ>` representation.
* **Downsampling Strategy**: The raw high-density point cloud is downsampled using a 3D Voxel Grid filter (`pcl::VoxelGrid`) with a leaf size of **0.25 m**. Raw point clouds contain millions of redundant points; voxelization reduces the search space and makes real-time 3D path planning feasible using $O(1)$ occupancy lookup via a hash map / `unordered_set<Index3D>`.
* **3D Spatial Representation**: Stacked 2D layers are strictly avoided. Full 3D index-based mapping ($X, Y, Z$) allows the drone to evaluate diagonal routes above and under obstacles.

#### Measured Map Metrics

* **World Bounding Box**:

  * $X \in [x_{\min}, x_{\max}]$ meters
  * $Y \in [y_{\min}, y_{\max}]$ meters
  * $Z \in [z_{\min}, z_{\max}]$ meters
* **Raw Points Count**: ~`X` points
* **Downsampled Voxel Count (0.25 m resolution)**: ~`Y` voxels
* **Inflated Voxel Count (0.60 m safety radius)**: ~`Z` voxels

#### Spatial Occupancy Query Interface

The node provides $O(1)$ spatial occupancy queries via:

* `bool isPointFree(double x, double y, double z)`
* `bool isOccupied(const Index3D& idx)`

*Test Case Output Log:*

```text
Query test at (0.0, 0.0, 1.0): FREE
Query test at (Shelf Center Coordinate): OCCUPIED
```

---

### Obstacle Inflation & Shelf Handling (0.5 pts)

#### Safety Radius Justification

The 3D obstacle inflation uses a configurable parameter `safety_radius = 0.60 m`. This value was chosen based on the following breakdown:

$$
\text{Safety Radius} =
\text{Drone Radius} +
\text{Position Controller Tolerance} +
\text{Safety Margin}
$$

* **Drone Physical Radius**: approximately $0.35\text{ m}$, accounting for the drone body and propeller span.
* **Position Controller Error Tolerance**: approximately $0.15\text{ m}$.
* **Safety Margin**: approximately $0.10\text{ m}$.
* **Total Safety Radius**: **$0.60\text{ m}**.

The controlled drone dimensions are approximately **45 × 45 × 15 cm**.

Inflation is computed in full 3D space using spherical Euclidean distance dilation:

$$
dx^2 + dy^2 + dz^2 \leq r_{\text{voxels}}^2
$$

#### Steel Racks / Shelving Handling

The steel shelving racks in the hangar (`regal.dae` link, approximately $6.5 \times 3.76 \times 5.59\text{ m}$) have open-mesh visuals in Gazebo, causing the point cloud scanner to sample sparse points with empty gaps between shelf layers.

* **Handling Strategy**: Dilation via `safety_radius = 0.60 m` fills the inner gaps between shelf layers, effectively solidifying the obstacle volume. This ensures that occupancy queries return `OCCUPIED` inside shelf gaps and forces the planner to route around the shelving structure.

---

### Mission CSV Waypoint Planning (A1.3)

The mission planner extends the map representation with sequential waypoint planning from CSV mission files.

#### Mission Input

Mission files are stored in:

```text
missions/*.csv
```

The CSV file contains the target coordinates that define the drone's mission. Waypoints are processed sequentially, with a collision-free path calculated between each consecutive pair of mission points.

The default mission is:

```text
missions/exam_example.csv
```

#### 3D A* Path Planning

For every pair of consecutive mission waypoints, the planner calculates a collision-free path through the 3D voxel occupancy map using the **A*** search algorithm.

The planner operates directly on the 3D voxel grid and considers neighboring voxels in all three spatial dimensions. Occupied and inflated voxels are treated as non-traversable.

The A* evaluation function is:

$$
f(n) = g(n) + h(n)
$$

where:

* $g(n)$ is the accumulated cost from the start voxel to node $n$.
* $h(n)$ is the heuristic estimate from node $n$ to the goal.
* $f(n)$ is the total estimated cost of the path through node $n$.

This allows the planner to find collision-free routes that can move around obstacles as well as above or below them.

#### Path Simplification

The raw A* result can contain a large number of adjacent voxel waypoints. To reduce unnecessary flight commands, the resulting path is simplified using **straight line-of-sight checks**.

If two non-adjacent path points can be connected by a collision-free straight segment through the inflated occupancy map, all intermediate points are removed.

This produces a shorter and smoother sequence of commands while preserving collision safety.

Conceptually:

```text
A* voxel path:

START -> . -> . -> . -> . -> . -> GOAL

After line-of-sight simplification:

START --------------------------> GOAL
```

where the direct segment is retained only when every relevant voxel along the segment is free.

#### Complete Mission Path

The simplified paths between all consecutive mission waypoints are combined into a single complete 3D flight plan.

The resulting path is published as:

```text
/planned_commands
```

using:

```text
nav_msgs/msg/Path
```

with `Transient Local` durability so the complete planned path remains available to RViz and other late-joining subscribers.

---

## Features

* **A1.1 Map Processing**: Loads 3D `.pcd` maps, calculates world bounding boxes, downsamples via PCL VoxelGrid, and provides $O(1)$ spatial occupancy queries (`isPointFree(x, y, z)`).
* **A1.2 3D Obstacle Inflation**: Expands obstacle boundaries in 3D space by a configurable `safety_radius` parameter to provide sufficient clearance for the drone.
* **A1.3 Mission Planning**: Parses sequential waypoints from Mission CSV files and plans collision-free 3D routes between them using A* search.
* **3D A* Path Planning**: Searches directly through the 3D voxel occupancy representation, allowing paths around, above, and below obstacles.
* **Path Simplification**: Removes unnecessary intermediate A* waypoints using collision-free straight line-of-sight checks.
* **Complete Mission Path**: Combines all sequential waypoint segments into a single simplified 3D flight plan.
* **Latching ROS Topics**: Publishes raw, downsampled, inflated, and planned path data with `Transient Local` durability for RViz visual debugging.

---

## Prerequisites

Ensure your ROS 2 workspace is built and sourced before running:

```bash
cd /home/user/ros2_ws
colcon build --packages-select my_drone_control
source install/setup.bash
```

---

## How to Run

### Step 1: Launch RViz Visualizer

Launch RViz2 with the pre-configured layout:

```bash
ros2 launch my_drone_control planner.launch.py
```

### Step 2: Run Map & Mission Path Planner

Run with the default mission file:

```bash
ros2 run my_drone_control map_planner_node
```

The default mission is:

```text
missions/exam_example.csv
```

### Step 3: Run with a Custom Mission

A different Mission CSV file can be provided using the `mission_path` ROS parameter:

```bash
ros2 run my_drone_control map_planner_node --ros-args -p mission_path:=/path/to/custom_mission.csv
```

---

## ROS 2 Interface

### Published Topics

| Topic               | Type                          | Durability      | Description                                                              |
| ------------------- | ----------------------------- | --------------- | ------------------------------------------------------------------------ |
| `/map_raw`          | `sensor_msgs/msg/PointCloud2` | Transient Local | Raw 3D point cloud loaded from the PCD map.                              |
| `/voxel_grid`       | `sensor_msgs/msg/PointCloud2` | Transient Local | Downsampled voxel grid ($0.25\text{ m}$ resolution).                     |
| `/voxel_inflated`   | `sensor_msgs/msg/PointCloud2` | Transient Local | 3D inflated voxel grid using the safety radius ($0.60\text{ m}$).        |
| `/planned_commands` | `nav_msgs/msg/Path`           | Transient Local | Complete simplified 3D flight plan generated from the mission waypoints. |

### Parameters

| Parameter       | Type     | Default                         | Description                                               |
| --------------- | -------- | ------------------------------- | --------------------------------------------------------- |
| `map_path`      | `string` | `.../maps/map.pcd`              | Path to the PCD map file.                                 |
| `mission_path`  | `string` | `.../missions/exam_example.csv` | Path to the Mission CSV file containing target waypoints. |
| `voxel_size`    | `double` | `0.25`                          | Voxel leaf size / spatial resolution in meters.           |
| `safety_radius` | `double` | `0.60`                          | Obstacle dilation safety radius in meters.                |

---

## Mission Planning Pipeline

The complete planning pipeline can be summarized as:

```text
             PCD Map
                |
                v
        Load Point Cloud
                |
                v
        Voxel Grid (0.25 m)
                |
                v
       3D Obstacle Inflation
          (0.60 m radius)
                |
                v
       3D Occupancy Map
                |
                |
        Mission CSV File
                |
                v
        Sequential Waypoints
                |
                v
          3D A* Planning
                |
                v
       Line-of-Sight Simplification
                |
                v
       Complete Mission Path
                |
                v
       /planned_commands
                |
                v
           Drone Controller
```

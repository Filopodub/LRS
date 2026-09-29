# My Drone Control — Map Processing and Path Planning

A ROS 2 package for 3D PCD map loading, voxel grid downsampling, 3D obstacle inflation, spatial occupancy querying, and path planning for autonomous drone navigation in hangar environments.

---

## 1. Documentation & Technical Specifications (Sub-assignment A1.1)

### Map Processing & Representation (1.5 pts)
- **Map Source**: FEI LRS Point Cloud (`maps/map.pcd`).
- **Loader**: Implemented using Point Cloud Library (`pcl::io::loadPCDFile`). Raw PCD points are loaded into a `pcl::PointCloud<pcl::PointXYZ>` representation.
- **Downsampling Strategy**: The raw high-density point cloud is downsampled using a 3D Voxel Grid filter (`pcl::VoxelGrid`) with a leaf size of **0.25 m**. Raw point clouds contain millions of redundant points; voxelization reduces search space dimensionality to make real-time 3D path planning feasible ($O(1)$ lookup via hash map `unordered_set<Index3D>`).
- **3D Spatial Representation**: Stacked 2D layers are strictly avoided. Full 3D index-based mapping ($X, Y, Z$) allows the drone to evaluate diagonal routes above and under obstacles.

#### Measured Map Metrics
- **World Bounding Box**:
  - $X \in [x_{\min}, x_{\max}]$ meters
  - $Y \in [y_{\min}, y_{\max}]$ meters
  - $Z \in [z_{\min}, z_{\max}]$ meters
- **Raw Points Count**: ~`X` points
- **Downsampled Voxel Count (0.25 m resolution)**: ~`Y` voxels
- **Inflated Voxel Count (0.60 m safety radius)**: ~`Z` voxels

#### Spatial Occupancy Query Interface
The node provides $O(1)$ spatial queries via:
- `bool isPointFree(double x, double y, double z)`
- `bool isOccupied(const Index3D& idx)`

*Test Case Output Log:*
```text
Query test at (0.0, 0.0, 1.0): FREE
Query test at (Shelf Center Coordinate): OCCUPIED

```

---

### Obstacle Inflation & Shelf Handling (0.5 pts)

#### Safety Radius Justification

The 3D obstacle inflation uses a configurable parameter `safety_radius = 0.60 m`. This value was chosen based on the following breakdown:


$$\text{Safety Radius} = \text{Drone Radius} + \text{Position Controller Tolerance} + \text{Safety Margin}$$

* **Drone Physical Radius**: $0.35\text{ m}$ (frame edge-to-edge with propellers)
* **Position Controller Error Tolerance**: $0.15\text{ m}$ (expected drift under turbulence/velocity overshoots)
* **Safety Margin**: $0.10\text{ m}$
* **Total Safety Radius**: **$0.60\text{ m}$**

Inflation is computed in full 3D space using spherical Euclidean distance dilation ($dx^2 + dy^2 + dz^2 \le r_{\text{voxels}}^2$).

#### Steel Racks / Shelving Handling

The steel shelving racks in the hangar (`regal.dae` link, $6.5 \times 3.76 \times 5.59\text{ m}$) have open-mesh visuals in Gazebo, causing the point cloud scanner to sample sparse points with empty gaps between shelves.

* **Handling Strategy**: Dilation via `safety_radius` ($0.60\text{ m}$) swallows inner gaps between shelf layers, solidifying the bounding volume of the racks. This ensures the occupancy query returns `OCCUPIED` inside shelf gaps, forcing the planner to route around the solid envelope.

---

## Features

* **A1.1 Map Processing**: Loads 3D `.pcd` maps, calculates world bounding boxes, downsamples via PCL VoxelGrid, and provides $O(1)$ spatial occupancy queries (`isPointFree(x, y, z)`).
* **A1.2 3D Obstacle Inflation**: Expands obstacle boundaries in 3D space by a configurable `safety_radius` parameter to prevent body collisions.
* **Latching ROS Topics**: Publishes raw, downsampled, and inflated voxel clouds with `Transient Local` durability for RViz visual debugging.

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

### Step 2: Run Map Node

In a second terminal, execute the map loader and obstacle inflator node:

```bash
ros2 run my_drone_control map_planner_node

```

---

## ROS 2 Interface

### Published Topics

| Topic | Type | Durability | Description |
| --- | --- | --- | --- |
| `/map_raw` | `sensor_msgs/msg/PointCloud2` | Transient Local | Raw 3D point cloud loaded from PCD map. |
| `/voxel_grid` | `sensor_msgs/msg/PointCloud2` | Transient Local | Downsampled voxel grid ($0.25\text{ m}$ resolution). |
| `/voxel_inflated` | `sensor_msgs/msg/PointCloud2` | Transient Local | 3D Inflated voxel grid using safety radius ($0.60\text{ m}$). |

### Parameters

| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `map_path` | `string` | `.../maps/map.pcd` | Absolute path to PCD file. |
| `voxel_size` | `double` | `0.25` | Voxel leaf size resolution in meters. |
| `safety_radius` | `double` | `0.60` | Dilation safety radius in meters. |

```

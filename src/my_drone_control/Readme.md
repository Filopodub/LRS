# My Drone Control — Map Processing and Path Planning

A ROS 2 package for 3D map loading, point cloud downsampling, obstacle inflation, and 3D path planning for autonomous drone navigation.

---

## Features

- **PCD Point Cloud Loading**: Reads 3D environment maps from `.pcd` files into ROS 2 `sensor_msgs/msg/PointCloud2`.
- **Voxel Grid Filtering**: Downsamples dense point clouds via PCL VoxelGrid filters for fast collision checking and planning.
- **Latched QoS**: Publishes using Transient Local durability so visualization tools (like RViz) and downstream nodes receive the map immediately upon subscribing.
- **Pre-configured Visualization**: Includes an RViz configuration and launch file to visualize raw maps and filtered voxel grids out of the box.

---

## Prerequisites

- **ROS 2** (Jazzy / Iron / Humble)
- **Point Cloud Library (PCL)**: `libpcl-dev`
- **ROS 2 Packages**:
  - `rclcpp`
  - `sensor_msgs`
  - `geometry_msgs`
  - `nav_msgs`
  - `pcl_conversions`

Install dependencies via `rosdep` (optional):
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

---

## Build Instructions

Build the package using `colcon`:

```bash
cd ~/ros2_ws
colcon build --packages-select my_drone_control
source install/setup.bash
```

---

## How to Run

### Option 1: All-in-One Launch (Recommended)

Launches both the `map_planner_node` and pre-configured RViz2 in a single command:

```bash
source install/setup.bash
ros2 launch my_drone_control planner.launch.py
```

### Option 2: Run Separately

1. **Launch the Map Planner Node:**
   ```bash
   source install/setup.bash
   ros2 run my_drone_control map_planner_node
   ```

2. **Launch RViz2:**
   ```bash
   source install/setup.bash
   rviz2 -d $(ros2 pkg prefix my_drone_control)/share/my_drone_control/config/planner_view.rviz
   ```

---

## ROS 2 Interface

### Published Topics

| Topic | Type | QoS Durability | Description |
|---|---|---|---|
| `/map_raw` | `sensor_msgs/msg/PointCloud2` | Transient Local | Raw 3D point cloud loaded from the PCD file. |
| `/voxel_grid` | `sensor_msgs/msg/PointCloud2` | Transient Local | Downsampled point cloud filtered using VoxelGrid. |

### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `map_path` | `string` | `.../maps/map.pcd` | Absolute path to the PCD map file. |
| `voxel_size` | `double` | `0.25` | Leaf size (in meters) for voxel grid downsampling. |

To run with custom parameters:
```bash
ros2 run my_drone_control map_planner_node --ros-args -p voxel_size:=0.5 -p map_path:=/path/to/custom_map.pcd
```

---

## RViz Visualization Notes

If configuring RViz manually without the provided launch/config file:

- **Fixed Frame**: Set to `map`.
- **Topic**: Subscribe to `/voxel_grid` or `/map_raw`.
- **QoS Durability Policy**: Set to **`Transient Local`** (since the map is published on startup).
- **Color Transformer**: Set to **`AxisColor`** or **`FlatColor`** (`pcl::PointXYZ` contains spatial coordinates without an intensity channel).
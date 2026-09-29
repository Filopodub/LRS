#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <fstream>
#include <sstream>
#include <cmath>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <queue>
#include <limits>
#include <chrono>
#include <algorithm>

struct Index3D {
  int x, y, z;
  bool operator==(const Index3D& o) const { return x == o.x && y == o.y && z == o.z; }
};

struct Index3DHash {
  std::size_t operator()(const Index3D& k) const {
    return ((std::hash<int>()(k.x) ^ (std::hash<int>()(k.y) << 1)) >> 1) ^ (std::hash<int>()(k.z) << 2);
  }
};

struct Waypoint {
  double x, y, z;
  std::string precision;
  std::string task;
};

struct AStarNode {
  Index3D idx;
  double g_score;
  double f_score;

  bool operator>(const AStarNode& other) const {
    return f_score > other.f_score;
  }
};

class MapPlannerNode : public rclcpp::Node {
public:
  MapPlannerNode() : Node("map_planner_node") {
    // ROS Parameters
    this->declare_parameter<std::string>("map_path", "/home/user/ros2_ws/src/my_drone_control/maps/map.pcd");
    this->declare_parameter<std::string>("mission_path", "/home/user/ros2_ws/src/my_drone_control/missions/exam_example.csv");
    this->declare_parameter<double>("voxel_size", 0.25);
    this->declare_parameter<double>("safety_radius", 0.60);

    // QoS Setup
    rclcpp::QoS qos_profile(1);
    qos_profile.transient_local();

    map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_raw", qos_profile);
    voxel_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("voxel_grid", qos_profile);
    inflated_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("voxel_inflated", qos_profile);
    
    // Target topic for controller
    planned_commands_pub_ = this->create_publisher<nav_msgs::msg::Path>("planned_commands", qos_profile);

    process_and_plan();
  }

  Index3D pointToIndex(double x, double y, double z) const {
    return {
      static_cast<int>(std::floor((x - min_x_) / voxel_size_)),
      static_cast<int>(std::floor((y - min_y_) / voxel_size_)),
      static_cast<int>(std::floor((z - min_z_) / voxel_size_))
    };
  }

  void indexToPoint(const Index3D& idx, double& x, double& y, double& z) const {
    x = min_x_ + (idx.x + 0.5) * voxel_size_;
    y = min_y_ + (idx.y + 0.5) * voxel_size_;
    z = min_z_ + (idx.z + 0.5) * voxel_size_;
  }

  bool isOccupied(const Index3D& idx) const {
    return inflated_grid_.find(idx) != inflated_grid_.end();
  }

private:
  std::vector<Waypoint> loadMissionCSV(const std::string& filepath) {
    std::vector<Waypoint> waypoints;
    std::ifstream file(filepath);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open mission CSV: %s", filepath.c_str());
      return waypoints;
    }

    std::string line;
    while (std::getline(file, line)) {
      if (line.empty() || line[0] == '#') continue; // Skip comments/empty lines
      std::stringstream ss(line);
      std::string x_s, y_s, z_s, prec, task;

      if (std::getline(ss, x_s, ',') && std::getline(ss, y_s, ',') &&
          std::getline(ss, z_s, ',') && std::getline(ss, prec, ',') &&
          std::getline(ss, task, ',')) {
        if (x_s == "x" || x_s == "X") continue; // Header row
        try {
          waypoints.push_back({std::stod(x_s), std::stod(y_s), std::stod(z_s), prec, task});
        } catch (...) {
          continue;
        }
      }
    }
    return waypoints;
  }

  void process_and_plan() {
    std::string map_path = this->get_parameter("map_path").as_string();
    std::string mission_path = this->get_parameter("mission_path").as_string();
    voxel_size_ = this->get_parameter("voxel_size").as_double();
    double safety_radius = this->get_parameter("safety_radius").as_double();

    // 1. Load Point Cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_path, *cloud) == -1) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load PCD: %s", map_path.c_str());
      return;
    }

    min_x_ = min_y_ = min_z_ = std::numeric_limits<double>::max();
    max_x_ = max_y_ = max_z_ = std::numeric_limits<double>::lowest();
    for (const auto& pt : cloud->points) {
      min_x_ = std::min(min_x_, static_cast<double>(pt.x));
      max_x_ = std::max(max_x_, static_cast<double>(pt.x));
      min_y_ = std::min(min_y_, static_cast<double>(pt.y));
      max_y_ = std::max(max_y_, static_cast<double>(pt.y));
      min_z_ = std::min(min_z_, static_cast<double>(pt.z));
      max_z_ = std::max(max_z_, static_cast<double>(pt.z));
    }

    // Voxel grid filtering & 3D inflation
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxels(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
    sor.filter(*cloud_voxels);

    for (const auto& pt : cloud_voxels->points) {
      occupied_grid_.insert(pointToIndex(pt.x, pt.y, pt.z));
    }

    int radius_voxels = std::ceil(safety_radius / voxel_size_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inflated_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& idx : occupied_grid_) {
      for (int dx = -radius_voxels; dx <= radius_voxels; ++dx) {
        for (int dy = -radius_voxels; dy <= radius_voxels; ++dy) {
          for (int dz = -radius_voxels; dz <= radius_voxels; ++dz) {
            if (dx*dx + dy*dy + dz*dz <= radius_voxels * radius_voxels) {
              Index3D inf_idx = {idx.x + dx, idx.y + dy, idx.z + dz};
              if (inflated_grid_.insert(inf_idx).second) {
                double x, y, z;
                indexToPoint(inf_idx, x, y, z);
                inflated_cloud->points.push_back(pcl::PointXYZ(x, y, z));
              }
            }
          }
        }
      }
    }

    publish_cloud(map_pub_, cloud);
    publish_cloud(voxel_pub_, cloud_voxels);
    publish_cloud(inflated_pub_, inflated_cloud);

    // 2. Load Mission CSV
    std::vector<Waypoint> mission = loadMissionCSV(mission_path);
    if (mission.size() < 2) {
      RCLCPP_ERROR(this->get_logger(), "Mission CSV needs at least 2 waypoints!");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Loaded mission with %zu waypoints from: %s", mission.size(), mission_path.c_str());

    // 3. Plan A* between sequential mission waypoints
    std::vector<Index3D> full_path;
    auto t_start = std::chrono::high_resolution_clock::now();

    for (size_t i = 0; i < mission.size() - 1; ++i) {
      Index3D start_idx = pointToIndex(mission[i].x, mission[i].y, mission[i].z);
      Index3D goal_idx = pointToIndex(mission[i+1].x, mission[i+1].y, mission[i+1].z);

      std::vector<Index3D> segment = plan3DAStar(start_idx, goal_idx);
      if (segment.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to find path between waypoint %zu and %zu!", i, i+1);
        return;
      }

      std::vector<Index3D> simplified_segment = simplifyPath(segment);
      if (full_path.empty()) {
        full_path.insert(full_path.end(), simplified_segment.begin(), simplified_segment.end());
      } else {
        full_path.insert(full_path.end(), simplified_segment.begin() + 1, simplified_segment.end());
      }
    }

    auto t_end = std::chrono::high_resolution_clock::now();
    double duration_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

    RCLCPP_INFO(this->get_logger(), "Full mission path calculated in %.2f ms! Total points: %zu", duration_ms, full_path.size());

    // 4. Publish to planned_commands topic
    publish_path(planned_commands_pub_, full_path);
  }

  std::vector<Index3D> plan3DAStar(const Index3D& start, const Index3D& goal) {
    if (isOccupied(start) || isOccupied(goal)) return {};

    auto heuristic = [](const Index3D& a, const Index3D& b) {
      return std::sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y) + (a.z - b.z)*(a.z - b.z));
    };

    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_set;
    std::unordered_set<Index3D, Index3DHash> closed_set;
    std::unordered_map<Index3D, Index3D, Index3DHash> came_from;
    std::unordered_map<Index3D, double, Index3DHash> g_score;

    g_score[start] = 0.0;
    open_set.push({start, 0.0, heuristic(start, goal)});

    while (!open_set.empty()) {
      Index3D current = open_set.top().idx;
      open_set.pop();

      if (current == goal) {
        std::vector<Index3D> path;
        Index3D curr = goal;
        while (!(curr == start)) {
          path.push_back(curr);
          curr = came_from[curr];
        }
        path.push_back(start);
        std::reverse(path.begin(), path.end());
        return path;
      }

      if (closed_set.find(current) != closed_set.end()) continue;
      closed_set.insert(current);

      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dz = -1; dz <= 1; ++dz) {
            if (dx == 0 && dy == 0 && dz == 0) continue;
            Index3D neighbor = {current.x + dx, current.y + dy, current.z + dz};
            if (isOccupied(neighbor) || closed_set.find(neighbor) != closed_set.end()) continue;

            double step_cost = std::sqrt(dx*dx + dy*dy + dz*dz);
            double tentative_g = g_score[current] + step_cost;

            if (g_score.find(neighbor) == g_score.end() || tentative_g < g_score[neighbor]) {
              came_from[neighbor] = current;
              g_score[neighbor] = tentative_g;
              open_set.push({neighbor, tentative_g, tentative_g + heuristic(neighbor, goal)});
            }
          }
        }
      }
    }
    return {};
  }

  bool isLineOfSightFree(const Index3D& p1, const Index3D& p2) const {
    int dx = std::abs(p2.x - p1.x), dy = std::abs(p2.y - p1.y), dz = std::abs(p2.z - p1.z);
    int max_steps = std::max({dx, dy, dz});
    if (max_steps == 0) return true;

    double x = p1.x, y = p1.y, z = p1.z;
    double x_inc = (p2.x - p1.x) / (double)max_steps;
    double y_inc = (p2.y - p1.y) / (double)max_steps;
    double z_inc = (p2.z - p1.z) / (double)max_steps;

    for (int i = 0; i <= max_steps; ++i) {
      if (isOccupied({static_cast<int>(std::round(x)),
                      static_cast<int>(std::round(y)),
                      static_cast<int>(std::round(z))})) {
        return false;
      }
      x += x_inc; y += y_inc; z += z_inc;
    }
    return true;
  }

  std::vector<Index3D> simplifyPath(const std::vector<Index3D>& path) const {
    if (path.size() <= 2) return path;
    std::vector<Index3D> simplified = {path.front()};
    size_t curr = 0;
    while (curr < path.size() - 1) {
      size_t furthest = curr + 1;
      for (size_t next = curr + 2; next < path.size(); ++next) {
        if (isLineOfSightFree(path[curr], path[next])) furthest = next;
      }
      simplified.push_back(path[furthest]);
      curr = furthest;
    }
    return simplified;
  }

  void publish_cloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.frame_id = "map";
    msg.header.stamp = this->now();
    pub->publish(msg);
  }

  void publish_path(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub, const std::vector<Index3D>& path) {
    nav_msgs::msg::Path msg;
    msg.header.frame_id = "map";
    msg.header.stamp = this->now();

    for (const auto& idx : path) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";
      indexToPoint(idx, pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
      pose.pose.orientation.w = 1.0;
      msg.poses.push_back(pose);
    }
    pub->publish(msg);
  }

  double voxel_size_;
  double min_x_, max_x_, min_y_, max_y_, min_z_, max_z_;
  std::unordered_set<Index3D, Index3DHash> occupied_grid_;
  std::unordered_set<Index3D, Index3DHash> inflated_grid_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr voxel_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr inflated_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr planned_commands_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MapPlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

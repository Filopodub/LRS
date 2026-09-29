#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cmath>
#include <vector>
#include <unordered_set>
#include <limits>
#include <algorithm>

// 3D Spatial Grid Index Structure
struct Index3D {
  int x, y, z;
  bool operator==(const Index3D& o) const { return x == o.x && y == o.y && z == o.z; }
};

struct Index3DHash {
  std::size_t operator()(const Index3D& k) const {
    return ((std::hash<int>()(k.x) ^ (std::hash<int>()(k.y) << 1)) >> 1) ^ (std::hash<int>()(k.z) << 2);
  }
};

class MapPlannerNode : public rclcpp::Node {
public:
  MapPlannerNode() : Node("map_planner_node") {
    // Parameters
    this->declare_parameter<std::string>("map_path", "/home/user/ros2_ws/src/my_drone_control/maps/map.pcd");
    this->declare_parameter<double>("voxel_size", 0.25);
    this->declare_parameter<double>("safety_radius", 0.60); // Drone radius + controller error + safety margin

    // QoS Setup (Transient Local for Latching Maps in RViz)
    rclcpp::QoS qos_profile(1);
    qos_profile.transient_local();

    map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_raw", qos_profile);
    voxel_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("voxel_grid", qos_profile);
    inflated_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("voxel_inflated", qos_profile);

    process_map();
  }

  // --- Sub-assignment A1.1: Map Occupancy Query Interface ---
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

  bool isPointFree(double x, double y, double z) const {
    return !isOccupied(pointToIndex(x, y, z));
  }

private:
  void process_map() {
    std::string map_path = this->get_parameter("map_path").as_string();
    voxel_size_ = this->get_parameter("voxel_size").as_double();
    double safety_radius = this->get_parameter("safety_radius").as_double();

    // -------------------------------------------------------------
    // A1.1: Map Loading, Downsampling, & Bounding Box Calculation
    // -------------------------------------------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_path, *cloud) == -1) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load PCD file from: %s", map_path.c_str());
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

    RCLCPP_INFO(this->get_logger(), "=== Map Bounding Box ===");
    RCLCPP_INFO(this->get_logger(), "X: [%.2f, %.2f] m", min_x_, max_x_);
    RCLCPP_INFO(this->get_logger(), "Y: [%.2f, %.2f] m", min_y_, max_y_);
    RCLCPP_INFO(this->get_logger(), "Z: [%.2f, %.2f] m", min_z_, max_z_);

    // Voxel Downsampling
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxels(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
    sor.filter(*cloud_voxels);

    for (const auto& pt : cloud_voxels->points) {
      occupied_grid_.insert(pointToIndex(pt.x, pt.y, pt.z));
    }

    RCLCPP_INFO(this->get_logger(), "Raw Downsampled Voxel Count: %zu", occupied_grid_.size());

    // -------------------------------------------------------------
    // A1.2: 3D Obstacle Inflation
    // -------------------------------------------------------------
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

    RCLCPP_INFO(this->get_logger(), "Inflated Voxel Count (Safety Radius = %.2f m): %zu",
                safety_radius, inflated_grid_.size());

    // Test spatial query
    double test_x = 0.0, test_y = 0.0, test_z = 1.0;
    RCLCPP_INFO(this->get_logger(), "Query test at (%.1f, %.1f, %.1f): %s",
                test_x, test_y, test_z, isPointFree(test_x, test_y, test_z) ? "FREE" : "OCCUPIED");

    // Publish PointClouds to ROS 2 Topics
    publish_cloud(map_pub_, cloud);
    publish_cloud(voxel_pub_, cloud_voxels);
    publish_cloud(inflated_pub_, inflated_cloud);
  }

  void publish_cloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub,
                     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.frame_id = "map";
    msg.header.stamp = this->now();
    pub->publish(msg);
  }

  double voxel_size_;
  double min_x_, max_x_, min_y_, max_y_, min_z_, max_z_;
  std::unordered_set<Index3D, Index3DHash> occupied_grid_;
  std::unordered_set<Index3D, Index3DHash> inflated_grid_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr voxel_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr inflated_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MapPlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

class MapPlannerNode : public rclcpp::Node {
public:
  MapPlannerNode() : Node("map_planner_node") {
    // Declare parameters
    this->declare_parameter<std::string>("map_path", "/home/user/ros2_ws/src/my_drone_control/maps/map.pcd");
    this->declare_parameter<double>("voxel_size", 0.25);

    // Create publishers (latching profile via transient local durability)
    rclcpp::QoS qos_profile(1);
    qos_profile.transient_local();

    map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_raw", qos_profile);
    voxel_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("voxel_grid", qos_profile);

    // Process and publish map
    load_and_publish_map();
  }

private:
  void load_and_publish_map() {
    std::string map_path = this->get_parameter("map_path").as_string();
    double voxel_size = this->get_parameter("voxel_size").as_double();

    // 1. Load PCD file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_path, *cloud) == -1) {
      RCLCPP_ERROR(this->get_logger(), "Couldn't read PCD file at: %s", map_path.c_str());
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Loaded raw PCD with %zu points", cloud->points.size());

    // 2. Downsample to Voxel Grid
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxels(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(voxel_size, voxel_size, voxel_size);
    sor.filter(*cloud_voxels);

    RCLCPP_INFO(this->get_logger(), "Downsampled Voxel Grid count: %zu points", cloud_voxels->points.size());

    // 3. Convert and publish Raw Cloud
    sensor_msgs::msg::PointCloud2 raw_msg;
    pcl::toROSMsg(*cloud, raw_msg);
    raw_msg.header.frame_id = "map";
    raw_msg.header.stamp = this->now();
    map_pub_->publish(raw_msg);

    // 4. Convert and publish Voxel Cloud
    sensor_msgs::msg::PointCloud2 voxel_msg;
    pcl::toROSMsg(*cloud_voxels, voxel_msg);
    voxel_msg.header.frame_id = "map";
    voxel_msg.header.stamp = this->now();
    voxel_pub_->publish(voxel_msg);

    RCLCPP_INFO(this->get_logger(), "Map topics published successfully!");
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr voxel_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MapPlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
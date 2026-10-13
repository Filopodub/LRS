#pragma once

#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace my_drone_control
{

struct PlannedCommand
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  std::string precision = "soft";
  std::string task = "-";
};

class FlightAssistant
{
public:
  explicit FlightAssistant(const std::string &frame_id = "map");

  bool takeoff(double altitude_m = 1.0);
  bool moveTo(double x, double y, double z, const std::string &precision = "soft");
  bool moveToMapPoint(double map_x, double map_y, double map_z, const std::string &precision = "soft");
  bool rotate(double yaw_deg);
  bool changeAltitude(double altitude_m, const std::string &precision = "soft");
  bool land();
  bool endMission();
  void attachNode(const rclcpp::Node::SharedPtr &node);
  void updateLocalPose(const geometry_msgs::msg::PoseStamped &pose);
  bool targetReached(const std::string &precision = "soft") const;

  bool execute(const PlannedCommand &command);

private:
  std::string frame_id_;
  bool armed_ = false;
  bool mission_active_ = false;
  double current_altitude_m_ = 0.0;
  double current_yaw_deg_ = 0.0;
  double target_x_ = 0.0;
  double target_y_ = 0.0;
  double local_x_ = 0.0;
  double local_y_ = 0.0;
  double local_z_ = 0.0;
  double local_yaw_deg_ = 0.0;
  bool pose_received_ = false;
  bool setpoint_control_active_ = false;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pub_;
  rclcpp::TimerBase::SharedPtr setpoint_timer_;
  double target_z_ = 0.0;
  double target_yaw_deg_ = 0.0;
};

}  // namespace my_drone_control

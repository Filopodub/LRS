#include "my_drone_control/flight_assistant.hpp"

#include <chrono>
#include <cmath>
#include <iostream>
#include <string>

namespace my_drone_control
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

double normalizeAngle(double angle_deg)
{
  while (angle_deg > 180.0) {
    angle_deg -= 360.0;
  }
  while (angle_deg < -180.0) {
    angle_deg += 360.0;
  }
  return angle_deg;
}

bool parseYawDegrees(const std::string &task, double &yaw_deg)
{
  if (task.rfind("yaw", 0) != 0) {
    return false;
  }

  const std::string value = task.substr(3);
  if (value.empty()) {
    return false;
  }

  try {
    yaw_deg = std::stod(value);
    return std::isfinite(yaw_deg);
  } catch (const std::exception &) {
    return false;
  }
}
}  // namespace

FlightAssistant::FlightAssistant(const std::string &frame_id)
: frame_id_(frame_id)
{
}

void FlightAssistant::attachNode(const rclcpp::Node::SharedPtr &node)
{
  node_ = node;
  setpoint_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/mavros/setpoint_position/local", 10);
  setpoint_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50), [this]() {
    if (!setpoint_control_active_) {
      return;
    }

    geometry_msgs::msg::PoseStamped setpoint;
    setpoint.header.stamp = node_->now();
    setpoint.header.frame_id = frame_id_;
    setpoint.pose.position.x = target_x_;
    setpoint.pose.position.y = target_y_;
    setpoint.pose.position.z = target_z_;
    const double yaw_rad = target_yaw_deg_ * kPi / 180.0;
    setpoint.pose.orientation.z = std::sin(yaw_rad / 2.0);
    setpoint.pose.orientation.w = std::cos(yaw_rad / 2.0);
    setpoint_pub_->publish(setpoint);
  });
}

void FlightAssistant::updateLocalPose(const geometry_msgs::msg::PoseStamped &pose)
{
  local_x_ = pose.pose.position.x;
  local_y_ = pose.pose.position.y;
  local_z_ = pose.pose.position.z;
  const auto &q = pose.pose.orientation;
  local_yaw_deg_ = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z)) * 180.0 / kPi;
  pose_received_ = true;
}

bool FlightAssistant::targetReached(const std::string &precision) const
{
  if (!pose_received_) {
    return false;
  }

  const double tolerance = precision == "hard" ? 0.15 : 0.30;
  const double position_error = std::sqrt(
    std::pow(local_x_ - target_x_, 2) +
    std::pow(local_y_ - target_y_, 2) +
    std::pow(local_z_ - target_z_, 2));
  const double yaw_tolerance = precision == "hard" ? 5.0 : 10.0;
  const double yaw_error = std::abs(normalizeAngle(local_yaw_deg_ - target_yaw_deg_));
  return position_error <= tolerance && yaw_error <= yaw_tolerance;
}

bool FlightAssistant::takeoff(double altitude_m)
{
  if (!std::isfinite(altitude_m) || altitude_m <= 0.0) {
    std::cout << "[FlightAssistant] invalid takeoff altitude: " << altitude_m << std::endl;
    return false;
  }

  std::cout << "[FlightAssistant] takeoff -> altitude " << altitude_m << " m in frame "
            << frame_id_ << std::endl;
  armed_ = true;
  mission_active_ = true;
  current_altitude_m_ = altitude_m;
  target_z_ = altitude_m;
  return true;
}

bool FlightAssistant::moveTo(double x, double y, double z, const std::string &precision)
{
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
    std::cout << "[FlightAssistant] invalid local position" << std::endl;
    return false;
  }

  std::cout << "[FlightAssistant] moveTo(" << x << ", " << y << ", " << z << ") precision="
            << precision << " in " << frame_id_ << std::endl;
  target_x_ = x;
  target_y_ = y;
  target_z_ = z;
  current_altitude_m_ = z;
  setpoint_control_active_ = true;
  return true;
}

bool FlightAssistant::moveToMapPoint(double map_x, double map_y, double map_z, const std::string &precision)
{
  const double local_x = map_x - map_origin_x_;
  const double local_y = map_y - map_origin_y_;
  const double local_z = map_z - map_origin_z_;

  std::cout << "[FlightAssistant] map->local: (" << map_x << ", " << map_y << ", " << map_z << ") -> ("
            << local_x << ", " << local_y << ", " << local_z << ")" << std::endl;
  return moveTo(local_x, local_y, local_z, precision);
}

bool FlightAssistant::rotate(double yaw_deg)
{
  if (!std::isfinite(yaw_deg)) {
    std::cout << "[FlightAssistant] invalid yaw: " << yaw_deg << std::endl;
    return false;
  }

  const double new_yaw = current_yaw_deg_ + yaw_deg;
  std::cout << "[FlightAssistant] rotate by " << yaw_deg << " deg"
            << " (current=" << current_yaw_deg_ << ", new=" << new_yaw << ")" << std::endl;
  current_yaw_deg_ = new_yaw;
  target_yaw_deg_ = new_yaw;
  return true;
}

bool FlightAssistant::changeAltitude(double altitude_m, const std::string &precision)
{
  if (!std::isfinite(altitude_m) || altitude_m < 0.0) {
    std::cout << "[FlightAssistant] invalid altitude: " << altitude_m << std::endl;
    return false;
  }

  std::cout << "[FlightAssistant] changeAltitude(" << altitude_m << ") precision="
            << precision << std::endl;
  current_altitude_m_ = altitude_m;
  target_z_ = altitude_m;
  setpoint_control_active_ = true;
  return true;
}

bool FlightAssistant::land()
{
  std::cout << "[FlightAssistant] land" << std::endl;
  current_altitude_m_ = 0.0;
  return true;
}

bool FlightAssistant::endMission()
{
  std::cout << "[FlightAssistant] endMission" << std::endl;
  mission_active_ = false;
  return true;
}

void FlightAssistant::setMapOrigin(double x, double y, double z)
{
  map_origin_x_ = x;
  map_origin_y_ = y;
  map_origin_z_ = z;
}

bool FlightAssistant::execute(const PlannedCommand &command)
{
  if (command.task == "takeoff") {
    return takeoff(command.z);
  }

  if (command.task.rfind("yaw", 0) == 0) {
    double yaw_delta = 0.0;
    if (!parseYawDegrees(command.task, yaw_delta)) {
      std::cout << "[FlightAssistant] invalid rotation command: " << command.task << std::endl;
      return false;
    }
    return rotate(yaw_delta);
  }

  if (command.task == "land") {
    return land();
  }

  if (command.task == "landtakeoff") {
    const double takeoff_altitude = current_altitude_m_ > 0.0 ? current_altitude_m_ : command.z;
    if (!land()) {
      return false;
    }
    return takeoff(takeoff_altitude);
  }

  if (command.task == "-" || command.task.empty()) {
    return moveToMapPoint(command.x, command.y, command.z, command.precision);
  }

  std::cout << "[FlightAssistant] invalid task: " << command.task << std::endl;
  return false;
}

}  // namespace my_drone_control

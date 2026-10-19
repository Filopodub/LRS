#pragma once

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/home_position.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "my_drone_control/flight_assistant.hpp"

namespace my_drone_control
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kEarthRadius = 6378137.0;  // WGS84 equatorial radius [m]

// -----------------------------------------------------------------------------
// ROS parameters (defaults, overridable with --ros-args -p name:=value)
// -----------------------------------------------------------------------------
struct OutdoorMissionParams
{
  double target_altitude = 5.0;      // [m] above launch (checked against rel_alt)
  double altitude_tolerance = 0.3;   // [m]
  int min_satellites = 6;            // minimum satellites for a usable fix
  std::string flight_mode = "GUIDED";
  double home_timeout = 5.0;         // [s] wait for home before using first valid fix
  double hover_time = 2.0;           // [s] hold after takeoff before SQUARE
  double square_side = 5.0;          // [m]
  double square_speed = 1.0;         // [m/s] speed of the setpoint along an edge
  double poi_east_offset = 2.5;      // [m] from launch in ENU
  double poi_north_offset = -2.5;    // [m] from launch in ENU
  double figure_eight_amplitude = 3.0;  // [m]
  double figure_eight_omega = 0.25;   // [rad/s], <= 0.35 keeps peak speed <= 1.5 m/s
  double position_tolerance = 0.3;   // [m] corner reached
  double yaw_tolerance = 5.0;        // [deg] nose vs. POI bearing at a corner
  double landing_tolerance = 0.3;    // [m] relative altitude to finish landing
};

struct Waypoint
{
  double x;
  double y;
  std::string name;
};

// -----------------------------------------------------------------------------
// Angle and coordinate helpers
// -----------------------------------------------------------------------------
inline double wrapDeg(double angle_deg)
{
  while (angle_deg > 180.0) {
    angle_deg -= 360.0;
  }
  while (angle_deg <= -180.0) {
    angle_deg += 360.0;
  }
  return angle_deg;
}

// Compass heading (0 = North, clockwise) -> ENU yaw (0 = East, counter-clockwise)
inline double headingToEnuYawDeg(double heading_deg)
{
  return wrapDeg(90.0 - heading_deg);
}

// ENU yaw (0 = East, counter-clockwise) from a quaternion
inline double yawFromQuaternionDeg(const geometry_msgs::msg::Quaternion &q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z)) * 180.0 / kPi;
}

// Flat-Earth (local tangent plane) approximation around the origin.
inline void geodeticToEnu(
  double lat, double lon, double alt,
  double lat0, double lon0, double alt0,
  double &east, double &north, double &up)
{
  const double d_lat = (lat - lat0) * kPi / 180.0;
  const double d_lon = (lon - lon0) * kPi / 180.0;
  east = d_lon * kEarthRadius * std::cos(lat0 * kPi / 180.0);
  north = d_lat * kEarthRadius;
  up = alt - alt0;
}

// Inverse of geodeticToEnu (same flat-Earth approximation).
inline void enuToGeodetic(
  double east, double north, double up,
  double lat0, double lon0, double alt0,
  double &lat, double &lon, double &alt)
{
  lat = lat0 + (north / kEarthRadius) * 180.0 / kPi;
  lon = lon0 + (east / (kEarthRadius * std::cos(lat0 * kPi / 180.0))) * 180.0 / kPi;
  alt = alt0 + up;
}

// -----------------------------------------------------------------------------
// Node
// -----------------------------------------------------------------------------
class OutdoorMission : public rclcpp::Node
{
public:
  OutdoorMission();
  void initializeFlightAssistant();

private:
  enum class MissionPhase {
    Preflight, Stabilize, Square, TransitToPoi, FigureEight,
    ReturnToLaunch, Landing, Complete, SafetyStop
  };

  void declareParameters();
  void createInterfaces();

  void setOrigin(double lat, double lon, double alt, const std::string &source);
  bool gpsFixUsable() const;

  void control_loop();
  void start_square();
  void square_step();
  void trajectory_step();
  void publish_setpoint(double x, double y, double z, double yaw_deg);
  void handle_abort(const std_srvs::srv::Trigger::Request::SharedPtr request,
                    std_srvs::srv::Trigger::Response::SharedPtr response);
  void enter_safety_stop(const std::string &reason);

  std::string phaseName() const;
  void report();

  void request_mode(const std::string &mode);
  void request_arm();
  void request_takeoff();

  // Parameters
  OutdoorMissionParams params_;

  // Mission state
  MissionPhase phase_{MissionPhase::Preflight};
  rclcpp::Time stabilize_start_;
  rclcpp::Time segment_start_time_;
  rclcpp::Time figure_eight_start_;
  rclcpp::Time next_mode_attempt_;
  rclcpp::Time state_update_time_;
  rclcpp::Time gps_update_time_;
  rclcpp::Time satellites_update_time_;
  rclcpp::Time pose_update_time_;
  std::vector<Waypoint> waypoints_;
  std::size_t waypoint_index_{0};
  double launch_x_{0.0};
  double launch_y_{0.0};
  double launch_east_{0.0};
  double launch_north_{0.0};
  double flight_z_{0.0};
  double poi_x_{0.0};
  double poi_y_{0.0};
  double segment_start_x_{0.0};
  double segment_start_y_{0.0};
  double setpoint_x_{0.0};
  double setpoint_y_{0.0};
  double desired_yaw_deg_{0.0};
  double yaw_error_deg_{0.0};
  double unwrapped_figure_yaw_deg_{0.0};

  // Sensor data
  mavros_msgs::msg::State state_;
  sensor_msgs::msg::NavSatFix gps_;
  geometry_msgs::msg::PoseStamped local_pose_;
  unsigned int satellites_{0};
  double compass_heading_deg_{0.0};
  double relative_altitude_{0.0};

  // Origin (home position or first valid fix)
  double origin_lat_{0.0};
  double origin_lon_{0.0};
  double origin_alt_{0.0};
  rclcpp::Time gps_wait_start_;

  // Data received flags
  bool state_received_{false};
  bool gps_received_{false};
  bool satellites_received_{false};
  bool origin_set_{false};
  bool gps_wait_started_{false};
  bool compass_received_{false};
  bool altitude_received_{false};
  bool pose_received_{false};

  // Arming / takeoff flags
  bool mode_request_pending_{false};
  bool arm_request_pending_{false};
  bool takeoff_request_pending_{false};
  bool mode_set_{false};
  bool armed_{false};
  bool takeoff_requested_{false};
  bool takeoff_complete_{false};
  bool landing_mode_requested_{false};
  bool abort_requested_{false};
  bool flight_started_{false};
  bool safety_stop_reported_{false};

  // ROS interfaces
  std::shared_ptr<FlightAssistant> assistant_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr satellites_sub_;
  rclcpp::Subscription<mavros_msgs::msg::HomePosition>::SharedPtr home_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr compass_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr altitude_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr abort_service_;
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr report_timer_;
  rclcpp::TimerBase::SharedPtr setpoint_timer_;
  rclcpp::TimerBase::SharedPtr trajectory_timer_;
};

}  // namespace my_drone_control

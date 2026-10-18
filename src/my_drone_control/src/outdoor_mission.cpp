#include "my_drone_control/outdoor_mission.hpp"

#include <algorithm>
#include <chrono>
#include <stdexcept>

using namespace std::chrono_literals;

namespace my_drone_control
{

OutdoorMission::OutdoorMission()
: Node("outdoor_mission")
{
  declareParameters();
  createInterfaces();
}

void OutdoorMission::declareParameters()
{
  auto &p = params_;
  p.target_altitude = declare_parameter<double>("target_altitude", p.target_altitude);
  p.altitude_tolerance = declare_parameter<double>("altitude_tolerance", p.altitude_tolerance);
  p.min_satellites = declare_parameter<int>("min_satellites", p.min_satellites);
  p.flight_mode = declare_parameter<std::string>("flight_mode", p.flight_mode);
  p.home_timeout = declare_parameter<double>("home_timeout", p.home_timeout);
  p.hover_time = declare_parameter<double>("hover_time", p.hover_time);
  p.square_side = declare_parameter<double>("square_side", p.square_side);
  p.square_speed = declare_parameter<double>("square_speed", p.square_speed);
  p.position_tolerance = declare_parameter<double>("position_tolerance", p.position_tolerance);
  p.yaw_tolerance = declare_parameter<double>("yaw_tolerance", p.yaw_tolerance);

  if (p.target_altitude <= 0.0 || p.altitude_tolerance <= 0.0 || p.min_satellites < 0 ||
    p.home_timeout <= 0.0 || p.hover_time < 0.0 || p.square_side <= 0.0 ||
    p.square_speed <= 0.0 || p.position_tolerance <= 0.0 || p.yaw_tolerance <= 0.0)
  {
    throw std::invalid_argument("outdoor mission parameters must be positive");
  }
}

void OutdoorMission::createInterfaces()
{
  state_sub_ = create_subscription<mavros_msgs::msg::State>(
    "mavros/state", 10,
    [this](const mavros_msgs::msg::State::SharedPtr message) {
      state_ = *message;
      state_received_ = true;
    });
  gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
    "mavros/global_position/global", rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::NavSatFix::SharedPtr message) {
      gps_ = *message;
      gps_received_ = true;
    });
  satellites_sub_ = create_subscription<std_msgs::msg::UInt32>(
    "mavros/global_position/raw/satellites", rclcpp::SensorDataQoS(),
    [this](const std_msgs::msg::UInt32::SharedPtr message) {
      satellites_ = message->data;
    });
  home_sub_ = create_subscription<mavros_msgs::msg::HomePosition>(
    "mavros/home_position/home", rclcpp::SensorDataQoS(),
    [this](const mavros_msgs::msg::HomePosition::SharedPtr message) {
      if (origin_set_) {
        return;
      }
      setOrigin(message->geo.latitude, message->geo.longitude, message->geo.altitude,
                "home position");
    });
  compass_sub_ = create_subscription<std_msgs::msg::Float64>(
    "mavros/global_position/compass_hdg", rclcpp::SensorDataQoS(),
    [this](const std_msgs::msg::Float64::SharedPtr message) {
      compass_heading_deg_ = message->data;
      compass_received_ = true;
    });
  altitude_sub_ = create_subscription<std_msgs::msg::Float64>(
    "mavros/global_position/rel_alt", rclcpp::SensorDataQoS(),
    [this](const std_msgs::msg::Float64::SharedPtr message) {
      relative_altitude_ = message->data;
      altitude_received_ = true;
    });

  mode_client_ = create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
  arm_client_ = create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
  takeoff_client_ = create_client<mavros_msgs::srv::CommandTOL>("mavros/cmd/takeoff");
  setpoint_pub_ = create_publisher<mavros_msgs::msg::PositionTarget>(
    "mavros/setpoint_raw/local", 10);

  control_timer_ = create_wall_timer(100ms, [this]() { control_loop(); });
  report_timer_ = create_wall_timer(1s, [this]() { report(); });
  setpoint_timer_ = create_wall_timer(50ms, [this]() { square_step(); });
}

void OutdoorMission::initializeFlightAssistant()
{
  assistant_ = std::make_shared<FlightAssistant>("map");
  assistant_->attachNode(shared_from_this());
  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "mavros/local_position/pose", rclcpp::SensorDataQoS(),
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
      assistant_->updateLocalPose(*message);
      local_pose_ = *message;
      pose_received_ = true;
    });
}

void OutdoorMission::setOrigin(double lat, double lon, double alt, const std::string &source)
{
  origin_lat_ = lat;
  origin_lon_ = lon;
  origin_alt_ = alt;
  origin_set_ = true;
  RCLCPP_INFO(get_logger(), "Origin set from %s: lat=%.7f lon=%.7f alt=%.2f m AMSL",
              source.c_str(), lat, lon, alt);
}

bool OutdoorMission::gpsFixUsable() const
{
  return gps_received_ &&
         gps_.status.status >= sensor_msgs::msg::NavSatStatus::STATUS_FIX &&
         satellites_ >= static_cast<unsigned int>(params_.min_satellites);
}

void OutdoorMission::control_loop()
{
  if (!state_received_ || !state_.connected) {
    return;
  }

  if (!gpsFixUsable()) {
    return;
  }

  // Prefer MAVROS home position; fall back to the first usable fix if home never arrives.
  if (!origin_set_) {
    if (!gps_wait_started_) {
      gps_wait_start_ = now();
      gps_wait_started_ = true;
    }
    if ((now() - gps_wait_start_).seconds() >= params_.home_timeout) {
      RCLCPP_WARN(get_logger(), "No home position after %.1f s, using current GPS fix",
                  params_.home_timeout);
      setOrigin(gps_.latitude, gps_.longitude, gps_.altitude, "first valid GPS fix");
    }
    return;
  }

  if (!mode_set_) {
    if (state_.mode == params_.flight_mode) {
      mode_set_ = true;
    } else if (!mode_request_pending_) {
      request_mode();
    }
    return;
  }

  if (!armed_) {
    if (state_.armed) {
      armed_ = true;
    } else if (!arm_request_pending_) {
      request_arm();
    }
    return;
  }

  if (!takeoff_requested_) {
    if (!takeoff_request_pending_) {
      request_takeoff();
    }
    return;
  }

  if (!takeoff_complete_) {
    if (altitude_received_ &&
      relative_altitude_ >= params_.target_altitude - params_.altitude_tolerance)
    {
      takeoff_complete_ = true;
      RCLCPP_INFO(get_logger(), "Takeoff complete: rel_alt=%.2f m (target %.2f m)",
                  relative_altitude_, params_.target_altitude);
      phase_ = MissionPhase::Stabilize;
      stabilize_start_ = now();
    }
    return;
  }

  if (phase_ == MissionPhase::Stabilize &&
    (now() - stabilize_start_).seconds() >= params_.hover_time)
  {
    start_square();
  }
}

void OutdoorMission::start_square()
{
  const double s = params_.square_side;
  // Corner offsets from launch in ENU metres (E, N): W1 = launch, W2 East, W3 South-East, W4 South
  const double offsets[4][2] = {{0.0, 0.0}, {s, 0.0}, {s, -s}, {0.0, -s}};

  waypoints_.clear();
  for (int i = 0; i < 4; ++i) {
    waypoints_.push_back({launch_x_ + offsets[i][0], launch_y_ + offsets[i][1],
                          "W" + std::to_string(i + 1)});
  }
  waypoints_.push_back(waypoints_.front());  // close the square back at W1

  // POI = centre of the square, distance from launch by Pythagoras
  const double poi_east = s / 2.0;
  const double poi_north = -s / 2.0;
  poi_x_ = launch_x_ + poi_east;
  poi_y_ = launch_y_ + poi_north;

  double lat = 0.0;
  double lon = 0.0;
  double alt = 0.0;
  enuToGeodetic(poi_east, poi_north, 0.0, origin_lat_, origin_lon_, origin_alt_, lat, lon, alt);
  RCLCPP_INFO(get_logger(),
              "POI: E=%.2f N=%.2f m from launch (distance %.2f m) -> lat=%.7f lon=%.7f",
              poi_east, poi_north, std::hypot(poi_east, poi_north), lat, lon);
  for (int i = 0; i < 4; ++i) {
    enuToGeodetic(offsets[i][0], offsets[i][1], params_.target_altitude,
                  origin_lat_, origin_lon_, origin_alt_, lat, lon, alt);
    RCLCPP_INFO(get_logger(), "%s: E=%.2f N=%.2f m from launch -> lat=%.7f lon=%.7f",
                waypoints_[i].name.c_str(), offsets[i][0], offsets[i][1], lat, lon);
  }

  waypoint_index_ = 0;
  segment_start_x_ = local_pose_.pose.position.x;
  segment_start_y_ = local_pose_.pose.position.y;
  segment_start_time_ = now();
  phase_ = MissionPhase::Square;
  RCLCPP_INFO(get_logger(), "Starting SQUARE (side %.1f m, speed %.2f m/s)",
              s, params_.square_speed);
}

// 20 Hz: move a setpoint along the current edge, yaw always towards the POI
void OutdoorMission::square_step()
{
  if ((phase_ != MissionPhase::Square && phase_ != MissionPhase::Hover) || !pose_received_) {
    return;
  }

  const double x = local_pose_.pose.position.x;
  const double y = local_pose_.pose.position.y;
  const Waypoint &target = waypoints_[waypoint_index_];

  const double dx = target.x - segment_start_x_;
  const double dy = target.y - segment_start_y_;
  const double length = std::hypot(dx, dy);
  const double elapsed = (now() - segment_start_time_).seconds();
  const double progress =
    length < 1e-3 ? 1.0 : std::min(1.0, params_.square_speed * elapsed / length);

  setpoint_x_ = segment_start_x_ + dx * progress;
  setpoint_y_ = segment_start_y_ + dy * progress;

  // Bearing from the actual position to the POI, recomputed every cycle (ENU)
  desired_yaw_deg_ = std::atan2(poi_y_ - y, poi_x_ - x) * 180.0 / kPi;
  yaw_error_deg_ = wrapDeg(yawFromQuaternionDeg(local_pose_.pose.orientation) - desired_yaw_deg_);

  publish_setpoint(setpoint_x_, setpoint_y_, flight_z_, desired_yaw_deg_);

  if (phase_ == MissionPhase::Hover) {
    return;
  }

  const double distance = std::hypot(target.x - x, target.y - y);
  if (progress >= 1.0 && distance <= params_.position_tolerance &&
    std::abs(yaw_error_deg_) <= params_.yaw_tolerance)
  {
    RCLCPP_INFO(get_logger(), "Reached %s (%zu/%zu), yaw error %.1f deg",
                target.name.c_str(), waypoint_index_ + 1, waypoints_.size(), yaw_error_deg_);
    segment_start_x_ = target.x;
    segment_start_y_ = target.y;
    segment_start_time_ = now();
    if (waypoint_index_ + 1 >= waypoints_.size()) {
      phase_ = MissionPhase::Hover;
      RCLCPP_INFO(get_logger(), "SQUARE complete, holding at %s", target.name.c_str());
      return;
    }
    ++waypoint_index_;
  }
}

void OutdoorMission::publish_setpoint(double x, double y, double z, double yaw_deg)
{
  using mavros_msgs::msg::PositionTarget;
  PositionTarget setpoint;
  setpoint.header.stamp = now();
  setpoint.header.frame_id = "map";
  setpoint.coordinate_frame = PositionTarget::FRAME_LOCAL_NED;  // MAVROS converts ENU -> NED
  setpoint.type_mask =
    PositionTarget::IGNORE_VX | PositionTarget::IGNORE_VY | PositionTarget::IGNORE_VZ |
    PositionTarget::IGNORE_AFX | PositionTarget::IGNORE_AFY | PositionTarget::IGNORE_AFZ |
    PositionTarget::IGNORE_YAW_RATE;
  setpoint.position.x = x;
  setpoint.position.y = y;
  setpoint.position.z = z;
  setpoint.yaw = static_cast<float>(yaw_deg * kPi / 180.0);  // ENU, radians
  setpoint_pub_->publish(setpoint);
}

std::string OutdoorMission::phaseName() const
{
  if (!state_received_ || !state_.connected) {
    return "WAIT_FCU";
  }
  if (!gpsFixUsable()) {
    return "WAIT_GPS_FIX";
  }
  if (!origin_set_) {
    return "WAIT_ORIGIN";
  }
  if (!mode_set_) {
    return "SET_MODE";
  }
  if (!armed_) {
    return "ARMING";
  }
  switch (phase_) {
    case MissionPhase::Stabilize:
      return "STABILIZE";
    case MissionPhase::Square:
      return "SQUARE";
    case MissionPhase::Hover:
      return "HOVER";
    default:
      return "TAKEOFF";
  }
}

void OutdoorMission::report()
{
  RCLCPP_INFO(get_logger(), "---- phase=%s mode=%s armed=%s ----",
              phaseName().c_str(), state_.mode.c_str(), state_.armed ? "yes" : "no");

  if (gps_received_) {
    RCLCPP_INFO(get_logger(),
                "GPS: lat=%.7f lon=%.7f alt=%.2f m AMSL | fix_status=%d sats=%u",
                gps_.latitude, gps_.longitude, gps_.altitude, gps_.status.status, satellites_);
  } else {
    RCLCPP_INFO(get_logger(), "GPS: no data");
  }

  if (gps_received_ && origin_set_) {
    double east = 0.0;
    double north = 0.0;
    double up = 0.0;
    geodeticToEnu(gps_.latitude, gps_.longitude, gps_.altitude,
                  origin_lat_, origin_lon_, origin_alt_,
                  east, north, up);
    RCLCPP_INFO(get_logger(), "ENU from origin (GPS): E=%.2f N=%.2f U=%.2f m", east, north, up);
  }

  if (pose_received_) {
    const auto &p = local_pose_.pose.position;
    const double yaw_enu_deg = yawFromQuaternionDeg(local_pose_.pose.orientation);
    RCLCPP_INFO(get_logger(), "Local pose (ENU): x=%.2f y=%.2f z=%.2f m yaw_enu=%.1f deg",
                p.x, p.y, p.z, yaw_enu_deg);
  }

  if (altitude_received_) {
    RCLCPP_INFO(get_logger(), "Relative altitude (above home): %.2f m", relative_altitude_);
  }

  if (compass_received_) {
    RCLCPP_INFO(get_logger(), "Compass: heading=%.1f deg (N, CW) -> yaw_enu=%.1f deg (E, CCW)",
                compass_heading_deg_, headingToEnuYawDeg(compass_heading_deg_));
  }

  if (phase_ == MissionPhase::Square || phase_ == MissionPhase::Hover) {
    RCLCPP_INFO(get_logger(),
                "SQUARE: target=%s setpoint=(%.2f, %.2f, %.2f) POI bearing=%.1f deg "
                "yaw error=%.1f deg",
                waypoints_[waypoint_index_].name.c_str(), setpoint_x_, setpoint_y_, flight_z_,
                desired_yaw_deg_, yaw_error_deg_);
  }
}

void OutdoorMission::request_mode()
{
  if (!mode_client_->service_is_ready()) {
    return;
  }
  auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
  request->base_mode = 0;
  request->custom_mode = params_.flight_mode;
  mode_request_pending_ = true;
  mode_client_->async_send_request(request,
    [this](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
      mode_request_pending_ = false;
      if (future.get()->mode_sent) {
        RCLCPP_INFO(get_logger(), "Flight mode request accepted: %s",
                    params_.flight_mode.c_str());
      }
    });
}

void OutdoorMission::request_arm()
{
  if (!arm_client_->service_is_ready()) {
    return;
  }
  auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
  request->value = true;
  arm_request_pending_ = true;
  arm_client_->async_send_request(request,
    [this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
      arm_request_pending_ = false;
      if (future.get()->success) {
        RCLCPP_INFO(get_logger(), "Arm request accepted");
      }
    });
}

void OutdoorMission::request_takeoff()
{
  if (!takeoff_client_->service_is_ready()) {
    return;
  }
  auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
  request->min_pitch = 0.0F;
  request->yaw = 0.0F;
  request->latitude = 0.0;
  request->longitude = 0.0;
  request->altitude = static_cast<float>(params_.target_altitude);
  // Launch point in the local frame: W1 of the square and reference for the flight altitude
  launch_x_ = local_pose_.pose.position.x;
  launch_y_ = local_pose_.pose.position.y;
  flight_z_ = local_pose_.pose.position.z + params_.target_altitude;
  takeoff_request_pending_ = true;
  takeoff_client_->async_send_request(request,
    [this](rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture future) {
      takeoff_request_pending_ = false;
      if (future.get()->success) {
        takeoff_requested_ = true;
        RCLCPP_INFO(get_logger(), "Takeoff request accepted; monitoring relative altitude");
      }
    });
}

}  // namespace my_drone_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<my_drone_control::OutdoorMission>();
    node->initializeFlightAssistant();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("outdoor_mission"), "%s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}

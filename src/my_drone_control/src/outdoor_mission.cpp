#include "my_drone_control/outdoor_mission.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
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
  p.poi_east_offset = declare_parameter<double>("poi_east_offset", p.poi_east_offset);
  p.poi_north_offset = declare_parameter<double>("poi_north_offset", p.poi_north_offset);
  p.figure_eight_amplitude = declare_parameter<double>(
    "figure_eight_amplitude", p.figure_eight_amplitude);
  p.figure_eight_omega = declare_parameter<double>("figure_eight_omega", p.figure_eight_omega);
  p.position_tolerance = declare_parameter<double>("position_tolerance", p.position_tolerance);
  p.yaw_tolerance = declare_parameter<double>("yaw_tolerance", p.yaw_tolerance);
  p.landing_tolerance = declare_parameter<double>("landing_tolerance", p.landing_tolerance);

  if (p.target_altitude <= 0.0 || p.altitude_tolerance <= 0.0 || p.min_satellites < 0 ||
    p.home_timeout <= 0.0 || p.hover_time < 0.0 || p.square_side <= 0.0 ||
    p.square_speed <= 0.0 || p.figure_eight_amplitude <= 0.0 ||
    p.figure_eight_omega <= 0.0 ||
    p.figure_eight_amplitude * p.figure_eight_omega * std::sqrt(2.0) > 1.5 ||
    p.position_tolerance <= 0.0 || p.yaw_tolerance <= 0.0 || p.landing_tolerance <= 0.0)
  {
    throw std::invalid_argument(
      "invalid outdoor mission parameters; omega must be in (0, 0.35] rad/s");
  }
}

void OutdoorMission::createInterfaces()
{
  state_sub_ = create_subscription<mavros_msgs::msg::State>(
    "mavros/state", 10,
    [this](const mavros_msgs::msg::State::SharedPtr message) {
      state_ = *message;
      state_received_ = true;
      state_update_time_ = now();
    });
  gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
    "mavros/global_position/global", rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::NavSatFix::SharedPtr message) {
      gps_ = *message;
      gps_received_ = true;
      gps_update_time_ = now();
    });
  satellites_sub_ = create_subscription<std_msgs::msg::UInt32>(
    "mavros/global_position/raw/satellites", rclcpp::SensorDataQoS(),
    [this](const std_msgs::msg::UInt32::SharedPtr message) {
      satellites_ = message->data;
      satellites_received_ = true;
      satellites_update_time_ = now();
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
  abort_service_ = create_service<std_srvs::srv::Trigger>(
    "~/abort",
    std::bind(&OutdoorMission::handle_abort, this, std::placeholders::_1, std::placeholders::_2));
  setpoint_pub_ = create_publisher<mavros_msgs::msg::PositionTarget>(
    "mavros/setpoint_raw/local", 10);

  control_timer_ = create_wall_timer(100ms, [this]() { control_loop(); });
  report_timer_ = create_wall_timer(1s, [this]() { report(); });
  setpoint_timer_ = create_wall_timer(50ms, [this]() { trajectory_step(); });
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
      pose_update_time_ = now();
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
         satellites_received_ &&
         (now() - gps_update_time_).seconds() <= 2.0 &&
         (now() - satellites_update_time_).seconds() <= 2.0 &&
         std::isfinite(gps_.latitude) && std::isfinite(gps_.longitude) &&
         std::isfinite(gps_.altitude) &&
         gps_.status.status >= sensor_msgs::msg::NavSatStatus::STATUS_FIX &&
         satellites_ >= static_cast<unsigned int>(params_.min_satellites);
}

void OutdoorMission::control_loop()
{
  if (phase_ == MissionPhase::Complete || phase_ == MissionPhase::SafetyStop) {
    return;
  }

  if (abort_requested_ && phase_ != MissionPhase::Landing) {
    phase_ = MissionPhase::Landing;
    landing_mode_requested_ = false;
    RCLCPP_WARN(get_logger(), "Abort requested; stopping setpoints and requesting LAND mode");
  }

  if (!state_received_ || !state_.connected ||
    (now() - state_update_time_).seconds() > 2.0)
  {
    if (flight_started_) {
      enter_safety_stop("MAVROS/FCU connection lost; setpoints stopped");
    }
    return;
  }

  if (flight_started_ && phase_ != MissionPhase::Landing &&
    mode_set_ && state_.mode != params_.flight_mode)
  {
    enter_safety_stop("flight mode changed externally; setpoints stopped for safety pilot");
    return;
  }

  if (flight_started_ && phase_ != MissionPhase::Landing && !state_.armed) {
    enter_safety_stop("vehicle disarmed externally; setpoints stopped");
    return;
  }

  if (phase_ == MissionPhase::Landing) {
    landing_mode_requested_ = state_.mode == "LAND";
    if (!landing_mode_requested_ && !mode_request_pending_ && now() >= next_mode_attempt_) {
      request_mode("LAND");
    }
    if (!state_.armed && altitude_received_ && relative_altitude_ <= params_.landing_tolerance) {
      phase_ = MissionPhase::Complete;
      RCLCPP_INFO(get_logger(), "Landing complete; mission finished");
    }
    return;
  }

  if (!gpsFixUsable()) {
    if (flight_started_ && phase_ != MissionPhase::Landing) {
      enter_safety_stop("usable GPS fix lost; setpoints stopped");
    }
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

  if (!pose_received_) {
    return;
  }

  if (!mode_set_) {
    if (state_.mode == params_.flight_mode) {
      mode_set_ = true;
    } else if (!mode_request_pending_ && now() >= next_mode_attempt_) {
      request_mode(params_.flight_mode);
    }
    return;
  }

  if (!armed_) {
    if (state_.armed) {
      armed_ = true;
      flight_started_ = true;
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
  // The four corners are relative ENU offsets from the captured launch position.
  const double offsets[4][2] = {{0.0, 0.0}, {s, 0.0}, {s, -s}, {0.0, -s}};

  waypoints_.clear();
  for (int i = 0; i < 4; ++i) {
    waypoints_.push_back({launch_x_ + offsets[i][0], launch_y_ + offsets[i][1],
                          "W" + std::to_string(i + 1)});
  }
  waypoints_.push_back(waypoints_.front());

  poi_x_ = launch_x_ + params_.poi_east_offset;
  poi_y_ = launch_y_ + params_.poi_north_offset;
  double launch_up = 0.0;
  geodeticToEnu(gps_.latitude, gps_.longitude, gps_.altitude,
                origin_lat_, origin_lon_, origin_alt_, launch_east_, launch_north_, launch_up);

  double lat = 0.0;
  double lon = 0.0;
  double alt = 0.0;
  enuToGeodetic(launch_east_ + params_.poi_east_offset,
                launch_north_ + params_.poi_north_offset, 0.0,
                origin_lat_, origin_lon_, origin_alt_, lat, lon, alt);
  RCLCPP_INFO(get_logger(),
              "POI: offset E=%.2f N=%.2f m from launch -> lat=%.7f lon=%.7f",
              params_.poi_east_offset, params_.poi_north_offset, lat, lon);
  for (int i = 0; i < 4; ++i) {
    enuToGeodetic(launch_east_ + offsets[i][0], launch_north_ + offsets[i][1],
                  params_.target_altitude,
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

void OutdoorMission::square_step()
{
  if (phase_ != MissionPhase::Square || !pose_received_) {
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
      phase_ = MissionPhase::TransitToPoi;
      RCLCPP_INFO(get_logger(), "SQUARE complete; transiting to figure-8 centre");
      return;
    }
    ++waypoint_index_;
  }
}

void OutdoorMission::trajectory_step()
{
  if (!state_received_ || !state_.connected || !state_.armed ||
    (now() - state_update_time_).seconds() > 2.0 ||
    state_.mode != params_.flight_mode || !pose_received_ ||
    (now() - pose_update_time_).seconds() > 1.0)
  {
    return;
  }

  if (phase_ == MissionPhase::Square) {
    square_step();
    return;
  }

  if (phase_ == MissionPhase::TransitToPoi) {
    const double distance = std::hypot(poi_x_ - local_pose_.pose.position.x,
                                      poi_y_ - local_pose_.pose.position.y);
    const double poi_yaw = std::atan2(
      poi_y_ - local_pose_.pose.position.y,
      poi_x_ - local_pose_.pose.position.x) * 180.0 / kPi;
    publish_setpoint(poi_x_, poi_y_, flight_z_, poi_yaw);
    if (distance <= params_.position_tolerance) {
      figure_eight_start_ = now();
      const double initial_tangent = std::atan2(1.0, 1.0) * 180.0 / kPi;
      unwrapped_figure_yaw_deg_ = initial_tangent;
      phase_ = MissionPhase::FigureEight;
      RCLCPP_INFO(get_logger(),
                  "At figure-8 centre; starting continuous trajectory (A=%.2f m, omega=%.3f rad/s)",
                  params_.figure_eight_amplitude, params_.figure_eight_omega);
    }
    return;
  }

  if (phase_ == MissionPhase::FigureEight) {
    const double elapsed = (now() - figure_eight_start_).seconds();
    const double t = std::min(2.0 * kPi, params_.figure_eight_omega * elapsed);
    const double amplitude = params_.figure_eight_amplitude;
    const double target_x = poi_x_ + amplitude * std::sin(t);
    const double target_y = poi_y_ + amplitude * std::sin(t) * std::cos(t);
    const double tangent_yaw = std::atan2(std::cos(2.0 * t), std::cos(t)) * 180.0 / kPi;
    unwrapped_figure_yaw_deg_ += wrapDeg(tangent_yaw - wrapDeg(unwrapped_figure_yaw_deg_));
    setpoint_x_ = target_x;
    setpoint_y_ = target_y;
    desired_yaw_deg_ = unwrapped_figure_yaw_deg_;
    publish_setpoint(target_x, target_y, flight_z_, desired_yaw_deg_);

    if (t >= 2.0 * kPi) {
      phase_ = MissionPhase::ReturnToLaunch;
      RCLCPP_INFO(get_logger(), "One figure-8 lap complete; returning to launch");
    }
    return;
  }

  if (phase_ == MissionPhase::ReturnToLaunch) {
    const double x = local_pose_.pose.position.x;
    const double y = local_pose_.pose.position.y;
    const double distance = std::hypot(launch_x_ - x, launch_y_ - y);
    if (distance > 0.05) {
      desired_yaw_deg_ = std::atan2(launch_y_ - y, launch_x_ - x) * 180.0 / kPi;
    }
    publish_setpoint(launch_x_, launch_y_, flight_z_, desired_yaw_deg_);
    if (distance <= params_.position_tolerance) {
      phase_ = MissionPhase::Landing;
      landing_mode_requested_ = false;
      next_mode_attempt_ = now();
      RCLCPP_INFO(get_logger(), "Returned to launch; requesting LAND mode");
    }
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
  if (phase_ == MissionPhase::Complete) {
    return "COMPLETE";
  }
  if (phase_ == MissionPhase::SafetyStop) {
    return "SAFETY_STOP";
  }
  if (phase_ == MissionPhase::Landing) {
    return "LANDING";
  }
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
    case MissionPhase::Preflight:
      return "PREFLIGHT";
    case MissionPhase::Stabilize:
      return "STABILIZE";
    case MissionPhase::Square:
      return "SQUARE";
    case MissionPhase::TransitToPoi:
      return "TRANSIT_TO_POI";
    case MissionPhase::FigureEight:
      return "FIGURE_EIGHT";
    case MissionPhase::ReturnToLaunch:
      return "RETURN_TO_LAUNCH";
    case MissionPhase::Landing:
      return "LANDING";
    case MissionPhase::Complete:
      return "COMPLETE";
    case MissionPhase::SafetyStop:
      return "SAFETY_STOP";
  }
  return "UNKNOWN";
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
    if (compass_received_) {
      RCLCPP_INFO(get_logger(), "Compass/pose yaw difference: %.1f deg",
                  wrapDeg(yaw_enu_deg - headingToEnuYawDeg(compass_heading_deg_)));
    }
  }

  if (altitude_received_) {
    RCLCPP_INFO(get_logger(), "Relative altitude (above home): %.2f m", relative_altitude_);
  }

  if (compass_received_) {
    RCLCPP_INFO(get_logger(), "Compass: heading=%.1f deg (N, CW) -> yaw_enu=%.1f deg (E, CCW)",
                compass_heading_deg_, headingToEnuYawDeg(compass_heading_deg_));
  }

  if (phase_ == MissionPhase::Square) {
    RCLCPP_INFO(get_logger(),
                "SQUARE: target=%s setpoint=(%.2f, %.2f, %.2f) POI bearing=%.1f deg "
                "yaw error=%.1f deg",
                waypoints_[waypoint_index_].name.c_str(), setpoint_x_, setpoint_y_, flight_z_,
                desired_yaw_deg_, yaw_error_deg_);
  } else if (phase_ == MissionPhase::FigureEight) {
    const double t = params_.figure_eight_omega * (now() - figure_eight_start_).seconds();
    RCLCPP_INFO(get_logger(),
                "FIGURE_EIGHT: t=%.2f/%.2f rad setpoint=(%.2f, %.2f, %.2f) tangent yaw=%.1f deg",
                t, 2.0 * kPi, setpoint_x_, setpoint_y_, flight_z_, desired_yaw_deg_);
  }
}

void OutdoorMission::request_mode(const std::string &mode)
{
  if (!mode_client_->service_is_ready()) {
    return;
  }
  auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
  request->base_mode = 0;
  request->custom_mode = mode;
  mode_request_pending_ = true;
  mode_client_->async_send_request(request,
    [this, mode](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
      mode_request_pending_ = false;
      if (future.get()->mode_sent) {
        RCLCPP_INFO(get_logger(), "Flight mode request accepted: %s", mode.c_str());
        next_mode_attempt_ = now() + rclcpp::Duration::from_seconds(2.0);
      } else {
        next_mode_attempt_ = now() + rclcpp::Duration::from_seconds(2.0);
        RCLCPP_WARN(get_logger(), "Flight mode request rejected: %s", mode.c_str());
      }
    });
}

void OutdoorMission::handle_abort(
  const std_srvs::srv::Trigger::Request::SharedPtr,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  if (!flight_started_) {
    phase_ = MissionPhase::Complete;
    response->success = true;
    response->message = "Preflight mission cancelled";
    return;
  }
  if (phase_ == MissionPhase::Complete || phase_ == MissionPhase::SafetyStop) {
    response->success = false;
    response->message = "Mission is already stopped";
    return;
  }
  abort_requested_ = true;
  response->success = true;
  response->message = "Abort accepted; LAND mode will be requested";
}

void OutdoorMission::enter_safety_stop(const std::string &reason)
{
  phase_ = MissionPhase::SafetyStop;
  if (!safety_stop_reported_) {
    RCLCPP_ERROR(get_logger(), "%s", reason.c_str());
    safety_stop_reported_ = true;
  }
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

}  

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

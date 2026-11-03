#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <array>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

class LocalizationSquareMission : public rclcpp::Node
{
public:
  LocalizationSquareMission()
  : Node("localization_square_mission")
  {
    start_x_ = declare_parameter<double>("start_x", -7.0);
    start_y_ = declare_parameter<double>("start_y", 13.0);
    altitude_ = declare_parameter<double>("altitude", 1.5);
    square_size_ = declare_parameter<double>("square_size", 2.0);
    speed_ = declare_parameter<double>("speed", 0.5);
    laps_ = declare_parameter<int>("laps", 1);
    startup_settle_s_ = declare_parameter<double>("startup_settle_s", 20.0);
    takeoff_hold_s_ = declare_parameter<double>("takeoff_hold_s", 12.0);
    leg_extra_hold_s_ = declare_parameter<double>("leg_extra_hold_s", 4.0);
    yaw_ = declare_parameter<double>("yaw", 1.57079632679);

    if (!std::isfinite(start_x_) || !std::isfinite(start_y_) ||
      !std::isfinite(altitude_) || altitude_ <= 0.0 ||
      !std::isfinite(square_size_) || square_size_ <= 0.0 ||
      !std::isfinite(speed_) || speed_ <= 0.0 || laps_ <= 0 ||
      !std::isfinite(startup_settle_s_) || startup_settle_s_ < 0.0 ||
      !std::isfinite(takeoff_hold_s_) || takeoff_hold_s_ <= 0.0 ||
      !std::isfinite(leg_extra_hold_s_) || leg_extra_hold_s_ < 0.0 ||
      !std::isfinite(yaw_))
    {
      throw std::invalid_argument("Square mission parameters must be finite and positive");
    }

    state_subscription_ = create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", 10,
      [this](const mavros_msgs::msg::State::SharedPtr message) {
        state_ = *message;
        state_received_ = true;
      });
    setpoint_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/mavros/setpoint_position/local", 10);
    mode_client_ = create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    arm_client_ = create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    takeoff_client_ = create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff");

    phase_started_ = now();
    next_request_time_ = now();
    timer_ = create_wall_timer(
      std::chrono::milliseconds(50), [this]() { tick(); });

    RCLCPP_WARN(
      get_logger(),
      "SITL characterization mission enabled: square relative to MAVROS local origin "
      "(%.1f, %.1f), altitude %.1f m, side %.1f m, %d lap(s). "
      "No MAVROS position feedback is used.",
      start_x_, start_y_, altitude_, square_size_, laps_);
  }

private:
  enum class Phase
  {
    WaitForFcu,
    Stabilizing,
    SetGuided,
    Arm,
    Takeoff,
    Square,
    Land,
    Complete
  };

  void tick()
  {
    if (phase_ == Phase::Complete) {
      return;
    }

    if (!state_received_ || !state_.connected) {
      return;
    }

    if (phase_ == Phase::WaitForFcu) {
      enter_phase(Phase::Stabilizing, "FCU connected; waiting for SITL/EKF startup");
    }

    if (phase_ == Phase::Land) {
      if (!state_.armed) {
        phase_ = Phase::Complete;
        RCLCPP_INFO(get_logger(), "Vehicle disarmed; square mission complete");
      } else if (state_.mode != "LAND") {
        request_mode("LAND");
      }
      return;
    }

    switch (phase_) {
      case Phase::Stabilizing:
        if ((now() - phase_started_).seconds() >= startup_settle_s_) {
          enter_phase(Phase::SetGuided, "Startup wait complete; requesting GUIDED");
        }
        break;
      case Phase::SetGuided:
        if (state_.mode == "GUIDED") {
          enter_phase(Phase::Arm, "GUIDED active; requesting arm");
        } else {
          request_mode("GUIDED");
        }
        break;
      case Phase::Arm:
        if (state_.armed) {
          enter_phase(Phase::Takeoff, "Armed; commanding takeoff position");
        } else {
          request_arm();
        }
        break;
      case Phase::Takeoff:
        if (!takeoff_requested_) {
          request_takeoff();
        } else if ((now() - phase_started_).seconds() >= takeoff_hold_s_) {
          if (!state_.armed) {
            phase_ = Phase::Complete;
            RCLCPP_ERROR(
              get_logger(),
              "FCU disarmed during takeoff hold; stopping without starting the square");
            return;
          }
          leg_index_ = 0;
          enter_phase(Phase::Square, "Takeoff hold complete; beginning square");
        }
        break;
      case Phase::Square:
        if (!state_.armed) {
          phase_ = Phase::Complete;
          RCLCPP_ERROR(get_logger(), "FCU disarmed during square; stopping mission");
          return;
        }
        publish_setpoint(target_for_current_phase());
        if ((now() - phase_started_).seconds() >= leg_duration_s()) {
          ++leg_index_;
          if (leg_index_ >= static_cast<std::size_t>(4 * laps_)) {
            enter_phase(Phase::Land, "Square complete; requesting LAND");
          } else {
            phase_started_ = now();
            RCLCPP_INFO(
              get_logger(), "Square leg %zu/%d: target (%.2f, %.2f, %.2f) ENU",
              leg_index_ + 1, 4 * laps_, target_for_current_phase()[0],
              target_for_current_phase()[1], altitude_);
          }
        }
        break;
      case Phase::WaitForFcu:
      case Phase::Land:
      case Phase::Complete:
        break;
    }
  }

  std::array<double, 3> target_for_current_phase() const
  {
    double x = start_x_;
    double y = start_y_;
    if (phase_ == Phase::Square) {
      switch (leg_index_ % 4) {
        case 0:
          y -= square_size_;
          break;
        case 1:
          x += square_size_;
          y -= square_size_;
          break;
        case 2:
          x += square_size_;
          break;
        default:
          break;
      }
    }
    return {x, y, altitude_};
  }

  double leg_duration_s() const
  {
    return square_size_ / speed_ + leg_extra_hold_s_;
  }

  void publish_setpoint(const std::array<double, 3> &target)
  {
    geometry_msgs::msg::PoseStamped setpoint;
    setpoint.header.stamp = now();
    setpoint.header.frame_id = "map";
    setpoint.pose.position.x = target[0];
    setpoint.pose.position.y = target[1];
    setpoint.pose.position.z = target[2];
    setpoint.pose.orientation.z = std::sin(yaw_ / 2.0);
    setpoint.pose.orientation.w = std::cos(yaw_ / 2.0);
    setpoint_publisher_->publish(setpoint);
  }

  void request_mode(const std::string &mode)
  {
    if (request_pending_ || now() < next_request_time_ || !mode_client_->service_is_ready()) {
      return;
    }
    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->base_mode = 0;
    request->custom_mode = mode;
    request_pending_ = true;
    mode_client_->async_send_request(request,
      [this, mode](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
        request_pending_ = false;
        next_request_time_ = now() + rclcpp::Duration::from_seconds(2.0);
        if (future.get()->mode_sent) {
          RCLCPP_INFO(get_logger(), "Mode request accepted: %s", mode.c_str());
        } else {
          RCLCPP_WARN(get_logger(), "Mode request rejected: %s", mode.c_str());
        }
      });
  }

  void request_arm()
  {
    if (request_pending_ || now() < next_request_time_ || !arm_client_->service_is_ready()) {
      return;
    }
    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = true;
    request_pending_ = true;
    arm_client_->async_send_request(request,
      [this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
        request_pending_ = false;
        if (future.get()->success) {
          next_request_time_ = now() + rclcpp::Duration::from_seconds(2.0);
          RCLCPP_INFO(get_logger(), "Arm request accepted");
        } else if (!arm_rejection_reported_) {
          next_request_time_ = now() + rclcpp::Duration::from_seconds(5.0);
          arm_rejection_reported_ = true;
          RCLCPP_WARN(
            get_logger(), "Arm not ready yet; retrying in 5 s while SITL pre-arm checks settle");
        } else {
          next_request_time_ = now() + rclcpp::Duration::from_seconds(5.0);
        }
      });
  }

  void request_takeoff()
  {
    if (request_pending_ || now() < next_request_time_ || !takeoff_client_->service_is_ready()) {
      return;
    }
    auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    request->min_pitch = 0.0F;
    request->yaw = 0.0F;
    request->latitude = 0.0;
    request->longitude = 0.0;
    request->altitude = static_cast<float>(altitude_);
    request_pending_ = true;
    takeoff_client_->async_send_request(request,
      [this](rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture future) {
        request_pending_ = false;
        next_request_time_ = now() + rclcpp::Duration::from_seconds(2.0);
        if (future.get()->success) {
          takeoff_requested_ = true;
          phase_started_ = now();
          RCLCPP_INFO(
            get_logger(), "MAVROS takeoff command accepted: target altitude %.2f m",
            altitude_);
        } else {
          RCLCPP_ERROR(
            get_logger(), "MAVROS takeoff command rejected (result %u); will retry",
            future.get()->result);
        }
      });
  }

  void enter_phase(Phase phase, const std::string &message)
  {
    phase_ = phase;
    phase_started_ = now();
    next_request_time_ = now();
    request_pending_ = false;
    RCLCPP_INFO(get_logger(), "%s", message.c_str());
  }

  double start_x_{-7.0};
  double start_y_{13.0};
  double altitude_{1.5};
  double square_size_{2.0};
  double speed_{0.5};
  double startup_settle_s_{20.0};
  double takeoff_hold_s_{12.0};
  double leg_extra_hold_s_{4.0};
  double yaw_{1.57079632679};
  int laps_{1};
  std::size_t leg_index_{0};
  Phase phase_{Phase::WaitForFcu};
  rclcpp::Time phase_started_{0, 0, RCL_ROS_TIME};
  rclcpp::Time next_request_time_{0, 0, RCL_ROS_TIME};
  bool state_received_{false};
  bool request_pending_{false};
  bool takeoff_requested_{false};
  bool arm_rejection_reported_{false};
  mavros_msgs::msg::State state_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_publisher_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<LocalizationSquareMission>());
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(rclcpp::get_logger("localization_square_mission"), "%s", exception.what());
  }
  rclcpp::shutdown();
  return 0;
}
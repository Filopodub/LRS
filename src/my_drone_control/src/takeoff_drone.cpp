#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/command_tol.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class TakeoffDrone : public rclcpp::Node
{
public:
  TakeoffDrone()
  : Node("takeoff_drone"), stage_(Stage::WAITING_FOR_CONNECTION)
  {
    target_altitude_ = declare_parameter<double>("target_altitude", 5.0);
    flight_mode_ = declare_parameter<std::string>("flight_mode", "GUIDED");
    altitude_tolerance_ = declare_parameter<double>("altitude_tolerance", 0.3);
    connection_timeout_ = declare_parameter<double>("connection_timeout", 30.0);
    takeoff_timeout_ = declare_parameter<double>("takeoff_timeout", 60.0);

    if (target_altitude_ <= 0.0 || altitude_tolerance_ <= 0.0 ||
      connection_timeout_ <= 0.0 || takeoff_timeout_ <= 0.0)
    {
      throw std::invalid_argument("takeoff parameters must be positive");
    }

    state_sub_ = create_subscription<mavros_msgs::msg::State>(
      "mavros/state", 10,
      [this](const mavros_msgs::msg::State::SharedPtr message) {
        state_ = *message;
        state_received_ = true;
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
    timer_ = create_wall_timer(100ms, [this]() { control_loop(); });
    start_time_ = now();
  }

private:
  enum class Stage { WAITING_FOR_CONNECTION, SETTING_MODE, ARMING, TAKING_OFF, CLIMBING };

  void control_loop()
  {
    if (stage_ == Stage::WAITING_FOR_CONNECTION) {
      if (state_received_ && state_.connected) {
        RCLCPP_INFO(get_logger(), "MAVROS connected; preparing takeoff to %.1f m", target_altitude_);
        stage_ = Stage::SETTING_MODE;
        stage_start_time_ = now();
      } else if ((now() - start_time_).seconds() > connection_timeout_) {
        abort("timed out waiting for MAVROS connection");
      }
      return;
    }

    if (!state_received_ || !state_.connected) {
      abort("MAVROS connection was lost");
      return;
    }

    if (stage_ == Stage::SETTING_MODE) {
      if (state_.mode == flight_mode_) {
        stage_ = Stage::ARMING;
        stage_start_time_ = now();
      } else if (!mode_request_pending_) {
        request_mode();
      } else if (timed_out()) {
        abort("timed out setting flight mode");
      }
      return;
    }

    if (stage_ == Stage::ARMING) {
      if (state_.armed) {
        stage_ = Stage::TAKING_OFF;
        stage_start_time_ = now();
      } else if (!arm_request_pending_) {
        request_arm();
      } else if (timed_out()) {
        abort("timed out arming the vehicle");
      }
      return;
    }

    if (stage_ == Stage::TAKING_OFF) {
      if (!takeoff_request_pending_) {
        request_takeoff();
      } else if (timed_out()) {
        abort("timed out sending the takeoff command");
      }
      return;
    }

    if (stage_ == Stage::CLIMBING) {
      if (altitude_received_ && relative_altitude_ >= target_altitude_ - altitude_tolerance_) {
        RCLCPP_INFO(get_logger(), "Takeoff complete at %.2f m relative altitude", relative_altitude_);
        rclcpp::shutdown();
      } else if (timed_out()) {
        abort("takeoff altitude was not reached");
      }
    }
  }

  void request_mode()
  {
    if (!mode_client_->service_is_ready()) {
      return;
    }
    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->base_mode = 0;
    request->custom_mode = flight_mode_;
    mode_request_pending_ = true;
    mode_client_->async_send_request(request,
      [this](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
        mode_request_pending_ = false;
        if (!future.get()->mode_sent) {
          abort("MAVROS rejected the flight mode request");
        } else {
          RCLCPP_INFO(get_logger(), "Flight mode request accepted: %s", flight_mode_.c_str());
        }
      });
  }

  void request_arm()
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
        if (!future.get()->success) {
          abort("MAVROS rejected the arm request");
        } else {
          RCLCPP_INFO(get_logger(), "Arm request accepted");
        }
      });
  }

  void request_takeoff()
  {
    if (!takeoff_client_->service_is_ready()) {
      return;
    }
    auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    request->min_pitch = 0.0F;
    request->yaw = 0.0F;
    request->latitude = 0.0;
    request->longitude = 0.0;
    request->altitude = static_cast<float>(target_altitude_);
    takeoff_request_pending_ = true;
    stage_ = Stage::CLIMBING;
    stage_start_time_ = now();
    takeoff_client_->async_send_request(request,
      [this](rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture future) {
        takeoff_request_pending_ = false;
        if (!future.get()->success) {
          abort("MAVROS rejected the takeoff request");
        } else {
          RCLCPP_INFO(get_logger(), "Takeoff request accepted; monitoring relative altitude");
        }
      });
  }

  bool timed_out() const
  {
    return (now() - stage_start_time_).seconds() >
      (stage_ == Stage::CLIMBING ? takeoff_timeout_ : connection_timeout_);
  }

  void abort(const std::string & reason)
  {
    if (aborted_) {
      return;
    }
    aborted_ = true;
    RCLCPP_ERROR(get_logger(), "Takeoff aborted: %s", reason.c_str());
    rclcpp::shutdown();
  }

  double target_altitude_;
  double altitude_tolerance_;
  double connection_timeout_;
  double takeoff_timeout_;
  std::string flight_mode_;
  Stage stage_;
  rclcpp::Time start_time_;
  rclcpp::Time stage_start_time_;
  mavros_msgs::msg::State state_;
  double relative_altitude_{0.0};
  bool state_received_{false};
  bool altitude_received_{false};
  bool mode_request_pending_{false};
  bool arm_request_pending_{false};
  bool takeoff_request_pending_{false};
  bool aborted_{false};
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr altitude_sub_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<TakeoffDrone>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("takeoff_drone"), "Startup failed: %s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
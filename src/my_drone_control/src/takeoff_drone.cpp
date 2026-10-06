#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/command_tol.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include "my_drone_control/flight_assistant.hpp"
#include "my_drone_control/msg/planned_mission.hpp"

using namespace std::chrono_literals;

class TakeoffDrone : public rclcpp::Node
{
public:
  TakeoffDrone()
  : Node("takeoff_drone"), stage_(Stage::WAITING_FOR_CONNECTION)
  {
    target_altitude_ = declare_parameter<double>("target_altitude", 2.0);
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
    mission_sub_ = create_subscription<my_drone_control::msg::PlannedMission>(
      "planned_commands", 10,
      [this](const my_drone_control::msg::PlannedMission::SharedPtr message) {
        if (mission_received_) {
          RCLCPP_WARN(get_logger(), "Ignoring duplicate mission");
          return;
        }
        mission_commands_ = message->commands;
        mission_received_ = true;
        RCLCPP_INFO(get_logger(), "Received mission with %zu commands", mission_commands_.size());
      });
    timer_ = create_wall_timer(100ms, [this]() { control_loop(); });
    start_time_ = now();
  }

  void initializeFlightAssistant()
  {
    assistant_ = std::make_shared<my_drone_control::FlightAssistant>("map");
    assistant_->setMapOrigin(0.0, 0.0, 0.0);
    assistant_->attachNode(shared_from_this());
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "mavros/local_position/pose", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
        assistant_->updateLocalPose(*message);
      });
  }

private:
  enum class Stage {
    WAITING_FOR_CONNECTION,
    SETTING_MODE,
    ARMING,
    TAKING_OFF,
    CLIMBING,
    EXECUTING_MISSION,
    MISSION_COMPLETE
  };

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
        if (mission_received_) {
          stage_ = Stage::EXECUTING_MISSION;
          start_next_command();
        } else {
          RCLCPP_INFO(get_logger(), "Waiting for planned mission");
        }
      } else if (timed_out()) {
        abort("takeoff altitude was not reached");
      }
      return;
    }

    if (stage_ == Stage::EXECUTING_MISSION && assistant_->targetReached(current_precision_)) {
      const auto &command = mission_commands_[mission_index_];
      if (command.task.rfind("yaw", 0) == 0 && !yaw_applied_) {
        double yaw_degrees = 0.0;
        try {
          yaw_degrees = std::stod(command.task.substr(3));
        } catch (const std::exception &) {
          abort("invalid yaw mission command");
          return;
        }
        if (!assistant_->rotate(yaw_degrees)) {
          abort("failed to apply yaw mission command");
          return;
        }
        yaw_applied_ = true;
        return;
      }

      RCLCPP_INFO(get_logger(), "Mission command %zu/%zu reached: %s",
                  mission_index_ + 1, mission_commands_.size(), command.task.c_str());
      ++mission_index_;
      start_next_command();
    }
  }

  void start_next_command()
  {
    while (mission_index_ < mission_commands_.size() &&
      mission_commands_[mission_index_].task == "takeoff")
    {
      ++mission_index_;
    }

    if (mission_index_ >= mission_commands_.size()) {
      RCLCPP_INFO(get_logger(), "Mission complete");
      stage_ = Stage::MISSION_COMPLETE;
      rclcpp::shutdown();
      return;
    }

    const auto &command = mission_commands_[mission_index_];
    current_precision_ = command.precision == "hard" ? "hard" : "soft";
    yaw_applied_ = false;

    if (command.task == "land") {
      assistant_->changeAltitude(0.0);
    } else if (command.task == "landtakeoff") {
      assistant_->changeAltitude(command.z);
    } else {
      assistant_->changeAltitude(command.z);
    }

    if (!assistant_->moveToMapPoint(command.x, command.y, command.z, current_precision_)) {
      abort("failed to set mission waypoint");
      return;
    }

    RCLCPP_INFO(get_logger(), "Executing mission command %zu/%zu: x=%.2f y=%.2f z=%.2f task=%s",
                mission_index_ + 1, mission_commands_.size(), command.x, command.y, command.z,
                command.task.c_str());
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
  std::shared_ptr<my_drone_control::FlightAssistant> assistant_;
  std::vector<my_drone_control::msg::PlannedCommand> mission_commands_;
  std::size_t mission_index_{0};
  std::string current_precision_{"soft"};
  bool mission_received_{false};
  bool yaw_applied_{false};
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<my_drone_control::msg::PlannedMission>::SharedPtr mission_sub_;
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
    auto node = std::make_shared<TakeoffDrone>();
    node->initializeFlightAssistant();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("takeoff_drone"), "Startup failed: %s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
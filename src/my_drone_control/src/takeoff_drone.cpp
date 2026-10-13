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
  : Node("takeoff_drone")
  {
    report_only_ = declare_parameter<bool>("report_only", false);
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
      "planned_commands", rclcpp::QoS(1).transient_local(),
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
    assistant_->attachNode(shared_from_this());
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "mavros/local_position/pose", rclcpp::SensorDataQoS(),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
        assistant_->updateLocalPose(*message);
        if (!location_reported_) {
          const auto &position = message->pose.position;
          const double map_x = position.y;
          const double map_y = -position.x;
          RCLCPP_INFO(get_logger(), "Converted start position (m): x=%.3f y=%.3f z=%.3f",
                      map_x, map_y, position.z);
          location_reported_ = true;
        }
      });
  }

private:
  void control_loop()
  {
    if (report_only_) {
      return;
    }

    if (!state_received_ || !state_.connected) {
      if ((now() - start_time_).seconds() > connection_timeout_) {
        return;
      }
      return;
    }

    if (!mode_set_) {
      if (state_.mode == flight_mode_) {
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
      if (!mission_received_) {
        return;
      }
      if (!takeoff_request_pending_) {
        request_takeoff();
      }
      return;
    }

    if (!takeoff_complete_) {
      if (altitude_received_ && relative_altitude_ >= active_takeoff_altitude_ - altitude_tolerance_) {
        takeoff_complete_ = true;
        if (mission_received_) {
          start_next_command();
        }
      }
      return;
    }

    if (!mission_received_ || mission_complete_) {
      return;
    }

    if (landtakeoff_phase_ == LandTakeoffPhase::Descending) {
      if (altitude_received_ && relative_altitude_ <= altitude_tolerance_) {
        assistant_->changeAltitude(mission_commands_[mission_index_].z);
        landtakeoff_phase_ = LandTakeoffPhase::Climbing;
      }
      return;
    }

    if (landtakeoff_phase_ == LandTakeoffPhase::Landing) {
      if (altitude_received_ && relative_altitude_ <= altitude_tolerance_) {
        landtakeoff_phase_ = LandTakeoffPhase::None;
        mission_complete_ = true;
        rclcpp::shutdown();
      }
      return;
    }

    if (landtakeoff_phase_ == LandTakeoffPhase::Climbing) {
      const auto &command = mission_commands_[mission_index_];
      if (altitude_received_ && relative_altitude_ >= command.z - altitude_tolerance_) {
        landtakeoff_phase_ = LandTakeoffPhase::None;
        ++mission_index_;
        start_next_command();
      }
      return;
    }

    if (assistant_->targetReached(current_precision_)) {
      const auto &command = mission_commands_[mission_index_];
      if (command.task.rfind("yaw", 0) == 0 && !yaw_applied_) {
        double yaw_degrees = 0.0;
        try {
          yaw_degrees = std::stod(command.task.substr(3));
        } catch (const std::exception &) {
          return;
        }
        assistant_->rotate(yaw_degrees);
        yaw_applied_ = true;
        return;
      }

      if (command.task == "landtakeoff") {
        assistant_->changeAltitude(0.0);
        landtakeoff_phase_ = LandTakeoffPhase::Descending;
        RCLCPP_INFO(get_logger(), "Landing before takeoff at mission command %zu",
                    mission_index_ + 1);
        return;
      }

      if (command.task == "land") {
        assistant_->changeAltitude(0.0);
        landtakeoff_phase_ = LandTakeoffPhase::Landing;
        RCLCPP_INFO(get_logger(), "Landing at final mission waypoint");
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
    if (mission_index_ >= mission_commands_.size()) {
      mission_complete_ = true;
      rclcpp::shutdown();
      return;
    }

    const auto &command = mission_commands_[mission_index_];
    current_precision_ = command.precision == "hard" ? "hard" : "soft";
    yaw_applied_ = false;

    assistant_->changeAltitude(command.z);

    assistant_->moveToMapPoint(command.x, command.y, command.z, current_precision_);

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
        if (future.get()->mode_sent) {
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
        if (future.get()->success) {
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
    active_takeoff_altitude_ = target_altitude_;
    if (mission_received_) {
      for (const auto &command : mission_commands_) {
        if (command.task == "takeoff" && command.z > 0.0) {
          active_takeoff_altitude_ = command.z;
          break;
        }
      }
    }
    request->altitude = static_cast<float>(active_takeoff_altitude_);
    takeoff_request_pending_ = true;
    takeoff_requested_ = true;
    stage_start_time_ = now();
    takeoff_client_->async_send_request(request,
      [this](rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture future) {
        takeoff_request_pending_ = false;
        if (future.get()->success) {
          RCLCPP_INFO(get_logger(), "Takeoff request accepted; monitoring relative altitude");
        }
      });
  }

  bool report_only_{true};
  bool location_reported_{false};
  double target_altitude_;
  double active_takeoff_altitude_{0.0};
  double altitude_tolerance_;
  double connection_timeout_;
  double takeoff_timeout_;
  std::string flight_mode_;
  rclcpp::Time start_time_;
  rclcpp::Time stage_start_time_;
  mavros_msgs::msg::State state_;
  double relative_altitude_{0.0};
  bool state_received_{false};
  bool altitude_received_{false};
  bool mode_request_pending_{false};
  bool arm_request_pending_{false};
  bool takeoff_request_pending_{false};
  bool mode_set_{false};
  bool armed_{false};
  bool takeoff_requested_{false};
  bool takeoff_complete_{false};
  bool mission_complete_{false};
  enum class LandTakeoffPhase { None, Descending, Climbing, Landing };
  LandTakeoffPhase landtakeoff_phase_{LandTakeoffPhase::None};
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
  } catch (const std::exception &) {
  }
  rclcpp::shutdown();
  return 0;
}
#ifndef TAKEOFF_DRONE_H_
#define TAKEOFF_DRONE_H_

#include <string>

#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/command_tol.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class TakeoffDrone : public rclcpp::Node
{
public:
  TakeoffDrone();

private:
  enum class Stage { WAITING_FOR_CONNECTION, SETTING_MODE, ARMING, TAKING_OFF, CLIMBING };

  void control_loop();
  void request_mode();
  void request_arm();
  void request_takeoff();
  bool timed_out() const;
  void abort(const std::string & reason);

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

#endif  // TAKEOFF_DRONE_H_
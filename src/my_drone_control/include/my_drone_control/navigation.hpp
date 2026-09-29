#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "my_drone_control/flight_assistant.hpp"
#include "my_drone_control/msg/planned_mission.hpp"

namespace my_drone_control
{

class NavigationNode : public rclcpp::Node
{
public:
  NavigationNode();

private:
  void plannedCommandsCallback(const my_drone_control::msg::PlannedMission::SharedPtr msg);
  void executeMission(const std::vector<my_drone_control::msg::PlannedCommand> &commands);

  std::shared_ptr<FlightAssistant> flight_assistant_;
  rclcpp::Subscription<my_drone_control::msg::PlannedMission>::SharedPtr planned_commands_sub_;
};

}  // namespace my_drone_control

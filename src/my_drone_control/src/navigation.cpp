#include "my_drone_control/navigation.hpp"

#include <string>
#include <vector>

namespace my_drone_control
{

NavigationNode::NavigationNode()
: Node("navigation_node")
{
  flight_assistant_ = std::make_shared<FlightAssistant>("map");

  planned_commands_sub_ = this->create_subscription<my_drone_control::msg::PlannedMission>(
    "planned_commands", 10,
    [this](const my_drone_control::msg::PlannedMission::SharedPtr msg) {
      this->plannedCommandsCallback(msg);
    });

  RCLCPP_INFO(this->get_logger(), "Navigation node started; waiting for planned_commands.");
}

void NavigationNode::plannedCommandsCallback(const my_drone_control::msg::PlannedMission::SharedPtr msg)
{
  if (!msg) {
    RCLCPP_WARN(this->get_logger(), "Received null planned mission.");
    return;
  }

  std::vector<my_drone_control::msg::PlannedCommand> commands(msg->commands.begin(), msg->commands.end());
  executeMission(commands);
}

void NavigationNode::executeMission(const std::vector<my_drone_control::msg::PlannedCommand> &commands)
{
  RCLCPP_INFO(this->get_logger(), "Executing mission with %zu commands", commands.size());

  for (const auto &command : commands) {
    RCLCPP_INFO(this->get_logger(),
                "Processing command: x=%.2f,y=%.2f,z=%.2f, precision=%s, task=%s",
                command.x, command.y, command.z, command.precision.c_str(), command.task.c_str());

    PlannedCommand next_command;
    next_command.x = command.x;
    next_command.y = command.y;
    next_command.z = command.z;
    next_command.precision = command.precision;
    next_command.task = command.task;

    if (!flight_assistant_->execute(next_command)) {
      RCLCPP_ERROR(this->get_logger(), "FlightAssistant rejected command: %s", command.task.c_str());
      break;
    }
  }

  flight_assistant_->endMission();
  RCLCPP_INFO(this->get_logger(), "Mission end reached.");
}

}  // namespace my_drone_control

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<my_drone_control::NavigationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

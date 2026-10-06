#include <rclcpp/rclcpp.hpp>

#include "my_drone_control/msg/planned_command.hpp"
#include "my_drone_control/msg/planned_mission.hpp"

#include <chrono>
#include <tuple>
#include <vector>

namespace my_drone_control
{

class MissionDemoNode : public rclcpp::Node
{
public:
  MissionDemoNode()
  : Node("mission_demo_node")
  {
    publisher_ = this->create_publisher<my_drone_control::msg::PlannedMission>("planned_commands", 10);

    auto mission = my_drone_control::msg::PlannedMission();
    const std::vector<std::tuple<double, double, double, std::string, std::string>> steps = {
      {0.00, 0.00, 2.00, "soft", "takeoff"},
      {1.00, 1.00, 1.00, "soft", "yaw180"},
      {2.65, 0.02, 2.00, "soft", "landtakeoff"},
      {0.00, 0.00, 3.00, "soft", "land"},
    };

    for (const auto &step : steps) {
      my_drone_control::msg::PlannedCommand command;
      command.x = std::get<0>(step);
      command.y = std::get<1>(step);
      command.z = std::get<2>(step);
      command.precision = std::get<3>(step);
      command.task = std::get<4>(step);
      mission.commands.push_back(command);
    }

    mission_ = mission;
    publish_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]() {
      if (publisher_->get_subscription_count() == 0) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Waiting for a planned_commands subscriber...");
        return;
      }

      publisher_->publish(mission_);
      RCLCPP_INFO(get_logger(), "Published mission with %zu commands to /planned_commands",
                  mission_.commands.size());
      publish_timer_->cancel();
    });
  }

private:
  rclcpp::Publisher<my_drone_control::msg::PlannedMission>::SharedPtr publisher_;
  my_drone_control::msg::PlannedMission mission_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace my_drone_control

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<my_drone_control::MissionDemoNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

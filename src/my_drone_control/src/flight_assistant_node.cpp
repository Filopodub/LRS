#include "my_drone_control/flight_assistant.hpp"

#include <iostream>
#include <string>
#include <vector>

int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;

  std::cout << "[flight_assistant_node] starting..." << std::endl;

  my_drone_control::FlightAssistant assistant("map");
  assistant.setMapOrigin(0.0, 0.0, 0.0);

  const std::vector<my_drone_control::PlannedCommand> demo = {
    {13.00, 7.00, 1.00, "soft", "takeoff"},
    {8.65, 2.02, 1.00, "soft", "-"},
    {4.84, 5.37, 2.00, "hard", "yaw180"},
    {2.08, 9.74, 1.75, "hard", "-"},
    {8.84, 6.90, 2.00, "hard", "landtakeoff"},
    {2.81, 8.15, 1.50, "soft", "yaw90"},
    {13.60, 1.50, 2.00, "hard", "land"}
  };

  for (const auto &command : demo) {
    assistant.execute(command);
  }

  assistant.endMission();
  std::cout << "[flight_assistant_node] ready" << std::endl;
  return 0;
}

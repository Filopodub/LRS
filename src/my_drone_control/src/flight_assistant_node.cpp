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

  const std::vector<my_drone_control::PlannedCommand> demo = {
      {13.00, 7.00, 1.00, "soft", "takeoff"},
      {9.00, 5.00, 1.00, "soft", "yaw180"},
      {10.65, 10.02, 2.00, "soft", "landtakeoff"},
      {13.00, 7.00, 1.00, "hard", "land"},

  };

  for (const auto &command : demo) {
    assistant.execute(command);
  }

  assistant.endMission();
  std::cout << "[flight_assistant_node] ready" << std::endl;
  return 0;
}

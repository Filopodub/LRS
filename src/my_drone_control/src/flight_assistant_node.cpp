#include "my_drone_control/flight_assistant.hpp"

#include <iostream>
#include <string>

int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;

  my_drone_control::FlightAssistant assistant("map");
  assistant.takeoff(1.0);
  assistant.rotate(180.0);
  assistant.changeAltitude(2.0, "hard");
  assistant.land();
  assistant.endMission();

  std::cout << "[flight_assistant_node] ready" << std::endl;
  return 0;
}

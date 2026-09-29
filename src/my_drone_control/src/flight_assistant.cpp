#include "my_drone_control/flight_assistant.hpp"

#include <iostream>
#include <string>

namespace my_drone_control
{

FlightAssistant::FlightAssistant(const std::string &frame_id)
: frame_id_(frame_id)
{
}

bool FlightAssistant::takeoff(double altitude_m)
{
  std::cout << "[FlightAssistant] takeoff -> altitude " << altitude_m << " m in frame "
            << frame_id_ << std::endl;
  armed_ = true;
  mission_active_ = true;
  current_altitude_m_ = altitude_m;
  return true;
}

bool FlightAssistant::moveTo(double x, double y, double z, const std::string &precision)
{
  std::cout << "[FlightAssistant] moveTo(" << x << ", " << y << ", " << z << ") precision="
            << precision << " in " << frame_id_ << std::endl;
  return true;
}

bool FlightAssistant::rotate(double yaw_deg)
{
  std::cout << "[FlightAssistant] rotate yaw=" << yaw_deg << " deg" << std::endl;
  current_yaw_deg_ = yaw_deg;
  return true;
}

bool FlightAssistant::changeAltitude(double altitude_m, const std::string &precision)
{
  std::cout << "[FlightAssistant] changeAltitude(" << altitude_m << ") precision="
            << precision << std::endl;
  current_altitude_m_ = altitude_m;
  return true;
}

bool FlightAssistant::land()
{
  std::cout << "[FlightAssistant] land" << std::endl;
  current_altitude_m_ = 0.0;
  return true;
}

bool FlightAssistant::endMission()
{
  std::cout << "[FlightAssistant] endMission" << std::endl;
  mission_active_ = false;
  return true;
}

bool FlightAssistant::execute(const PlannedCommand &command)
{
  if (command.task == "takeoff") {
    return takeoff(command.z);
  }

  if (command.task == "yaw180" || command.task == "yaw90" || command.task == "yaw270") {
    double desired_yaw = 0.0;
    if (command.task == "yaw180") {
      desired_yaw = 180.0;
    } else if (command.task == "yaw90") {
      desired_yaw = 90.0;
    } else if (command.task == "yaw270") {
      desired_yaw = 270.0;
    }
    return rotate(desired_yaw);
  }

  if (command.task == "land") {
    return land();
  }

  if (command.task == "landtakeoff") {
    if (!land()) {
      return false;
    }
    return takeoff(command.z);
  }

  if (command.task == "-" || command.task.empty()) {
    return moveTo(command.x, command.y, command.z, command.precision);
  }

  return moveTo(command.x, command.y, command.z, command.precision);
}

}  // namespace my_drone_control

#include "my_drone_control/flight_assistant.hpp"

#include <iostream>
#include <string>

namespace my_drone_control
{
namespace
{
bool parseYawDegrees(const std::string &task, double &yaw_deg)
{
  if (task.rfind("yaw", 0) != 0) {
    return false;
  }

  const std::string value = task.substr(3);
  if (value.empty()) {
    return false;
  }

  try {
    yaw_deg = std::stod(value);
    return true;
  } catch (const std::exception &) {
    return false;
  }
}
}  // namespace

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

bool FlightAssistant::moveToMapPoint(double map_x, double map_y, double map_z, const std::string &precision)
{
  const double local_x = map_x - map_origin_x_;
  const double local_y = map_y - map_origin_y_;
  const double local_z = map_z - map_origin_z_;

  std::cout << "[FlightAssistant] map->local: (" << map_x << ", " << map_y << ", " << map_z << ") -> ("
            << local_x << ", " << local_y << ", " << local_z << ")" << std::endl;
  return moveTo(local_x, local_y, local_z, precision);
}

bool FlightAssistant::rotate(double yaw_deg)
{
  const double new_yaw = current_yaw_deg_ + yaw_deg;
  std::cout << "[FlightAssistant] rotate by " << yaw_deg << " deg"
            << " (current=" << current_yaw_deg_ << ", new=" << new_yaw << ")" << std::endl;
  current_yaw_deg_ = new_yaw;
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

void FlightAssistant::setMapOrigin(double x, double y, double z)
{
  map_origin_x_ = x;
  map_origin_y_ = y;
  map_origin_z_ = z;
}

bool FlightAssistant::execute(const PlannedCommand &command)
{
  if (command.task == "takeoff") {
    return takeoff(command.z);
  }

  if (command.task.rfind("yaw", 0) == 0) {
    double yaw_delta = 0.0;
    if (!parseYawDegrees(command.task, yaw_delta)) {
      std::cout << "[FlightAssistant] invalid rotation command: " << command.task << std::endl;
      return false;
    }
    return rotate(yaw_delta);
  }

  if (command.task == "land") {
    return land();
  }

  if (command.task == "landtakeoff") {
    if (!land()) {
      return false;
    }
    return takeoff(current_altitude_m_ == 0.0 ? command.z : current_altitude_m_);
  }

  if (command.task == "-" || command.task.empty()) {
    return moveToMapPoint(command.x, command.y, command.z, command.precision);
  }

  return moveToMapPoint(command.x, command.y, command.z, command.precision);
}

}  // namespace my_drone_control

#pragma once

#include <string>

namespace my_drone_control
{

struct PlannedCommand
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  std::string precision = "soft";
  std::string task = "-";
};

class FlightAssistant
{
public:
  explicit FlightAssistant(const std::string &frame_id = "map");

  bool takeoff(double altitude_m = 1.0);
  bool moveTo(double x, double y, double z, const std::string &precision = "soft");
  bool rotate(double yaw_deg);
  bool changeAltitude(double altitude_m, const std::string &precision = "soft");
  bool land();
  bool endMission();

  bool execute(const PlannedCommand &command);

private:
  std::string frame_id_;
  bool armed_ = false;
  bool mission_active_ = false;
  double current_altitude_m_ = 0.0;
  double current_yaw_deg_ = 0.0;
};

}  // namespace my_drone_control

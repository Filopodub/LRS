#!/usr/bin/env bash
# Assignment 2 start script: localization world, SITL, MAVROS, sensor simulation,
# EKF, plotter and the square + figure-8 mission flown on the EKF estimate.
# Keys in this terminal: 'e' = toggle GPS (simulated outage), 'o' = inject GPS outlier,
# 'q' = quit the key loop.

set -e

REPO_DIR="${REPO_DIR:-$HOME/Documents/github/LRS-URK}"
ROS2_WS="${ROS2_WS:-$HOME/ros2_ws}"
SEED="${SEED:-42}"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
WS_SETUP="$ROS2_WS/install/setup.bash"

for f in "$ROS_SETUP" "$WS_SETUP"; do
  if [ ! -f "$f" ]; then
    echo "Setup file not found: $f (build the workspace first)"
    exit 1
  fi
done
if [ ! -d "$REPO_DIR" ]; then
  echo "LRS-URK checkout not found: $REPO_DIR (set REPO_DIR=...)"
  exit 1
fi

source "$ROS_SETUP"
source "$WS_SETUP"

# One launch starts Gazebo, SITL, MAVROS, sensors, EKF, plotter and the mission.
gnome-terminal --title="Assignment 2 stack" \
  -- bash -ic "source '$ROS_SETUP'; source '$WS_SETUP'; cd '$ROS2_WS'; \
ros2 launch my_drone_control localization_plots.launch.py \
repo_dir:='$REPO_DIR' seed:=$SEED run_mission:=true; exec bash"

# ---------------------------------------------------------------------------
# Keyboard control of the sensor simulation
# ---------------------------------------------------------------------------
gps_enabled=true
echo "Keys: 'e' = disable/enable GPS, 'o' = inject one GPS outlier, 'q' = quit."
while IFS= read -rsn1 key; do
  case "$key" in
    e|E)
      if $gps_enabled; then want=false; else want=true; fi
      if ros2 service call /localization/set_gps_enabled std_srvs/srv/SetBool \
        "{data: $want}" >/dev/null 2>&1; then
        gps_enabled=$want
        echo "[$(date +%T)] GPS enabled -> $gps_enabled"
      else
        echo "  service call failed (is localization_sensor_sim running yet?)"
      fi
      ;;
    o|O)
      if ros2 service call /localization/inject_gps_outlier std_srvs/srv/Trigger "{}" \
        >/dev/null 2>&1; then
        echo "[$(date +%T)] GPS outlier injected"
      else
        echo "  service call failed (is localization_sensor_sim running yet?)"
      fi
      ;;
    q|Q) break ;;
  esac
done

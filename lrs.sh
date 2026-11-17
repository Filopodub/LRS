#!/usr/bin/env bash

set -e

REPO_DIR="$HOME/LRS-URK"
ROS2_WS="$HOME/ros2_ws"

gnome-terminal --working-directory="$REPO_DIR" \
  -- bash -ic './scripts/run_gazebo.sh; exec bash'

gnome-terminal --working-directory="$REPO_DIR" \
  -- bash -ic './scripts/run_sitl.sh; exec bash'

gnome-terminal --working-directory="$REPO_DIR" \
  -- bash -ic './scripts/run_mavros.sh; exec bash'

gnome-terminal \
  -- bash -ic "cd '$ROS2_WS' && source install/setup.bash && exec bash"

# ---------------------------------------------------------------------------
# Keyboard control: 'e' toggles the localization GPS (simulated outage), 'q' quits.
# ---------------------------------------------------------------------------
source /opt/ros/jazzy/setup.bash
[ -f "$ROS2_WS/install/setup.bash" ] && source "$ROS2_WS/install/setup.bash"

gps_enabled=true
echo "Press 'e' to disable/enable GPS (/localization/set_gps_enabled), 'q' to quit."
while IFS= read -rsn1 key; do
  case "$key" in
    e|E)
      if $gps_enabled; then want=false; else want=true; fi
      if ros2 service call /localization/set_gps_enabled std_srvs/srv/SetBool \
        "{data: $want}" >/dev/null 2>&1; then
        gps_enabled=$want
        echo "[$(date +%T)] GPS enabled -> $gps_enabled"
      else
        echo "  service call failed (is localization_sensor_sim running?)"
      fi
      ;;
    q|Q) break ;;
  esac
done

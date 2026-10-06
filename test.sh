#!/usr/bin/env bash

set -e

REPO_DIR="${REPO_DIR:-$HOME/Documents/github/LRS-URK}"
ROS2_WS="${ROS2_WS:-$HOME/ros2_ws}"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
WS_SETUP="$ROS2_WS/install/setup.bash"

if [ ! -f "$ROS_SETUP" ]; then
  echo "ROS2 setup file not found: $ROS_SETUP"
  exit 1
fi

if [ -f "$WS_SETUP" ]; then
  source "$WS_SETUP"
fi

for script in run_gazebo.sh run_sitl.sh run_mavros.sh; do
  if [ -f "$REPO_DIR/scripts/$script" ]; then
    gnome-terminal --working-directory="$REPO_DIR" \
      -- bash -ic "source '$ROS_SETUP'; source '$WS_SETUP'; cd '$REPO_DIR'; ./scripts/$script; exec bash"
  else
    gnome-terminal --working-directory="$REPO_DIR" \
      -- bash -ic "source '$ROS_SETUP'; source '$WS_SETUP'; cd '$REPO_DIR'; echo 'Missing script: $script'; exec bash"
  fi
done

sleep 5

gnome-terminal \
  -- bash -ic "source '$ROS_SETUP'; source '$WS_SETUP'; cd '$ROS2_WS'; ros2 run my_drone_control flight_assistant_node; exec bash"

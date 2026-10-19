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

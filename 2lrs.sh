#!/usr/bin/env bash

set -e

REPO_DIR="${REPO_DIR:-$HOME/Documents/github/LRS-URK}"
ROS2_WS="${ROS2_WS:-$HOME/ros2_ws}"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
WS_SETUP="$ROS2_WS/install/setup.bash"

# Outdoor world + matching SITL home (must equal <spherical_coordinates> in the world file)
WORLD="fei_lrs_outdoor.world"
HOME_LOCATION="${HOME_LOCATION:-48.15144444,17.07375000,150,0}"

if [ ! -f "$ROS_SETUP" ]; then
  echo "ROS2 setup file not found: $ROS_SETUP"
  exit 1
fi

if [ -f "$WS_SETUP" ]; then
  source "$WS_SETUP"
fi

declare -a COMMANDS=(
  "./scripts/run_gazebo.sh $WORLD"
  "HOME_LOCATION='$HOME_LOCATION' ./scripts/run_sitl.sh"
  "./scripts/run_mavros.sh"
)

for cmd in "${COMMANDS[@]}"; do
  script="${cmd##*./scripts/}"
  script="${script%% *}"
  if [ -f "$REPO_DIR/scripts/$script" ]; then
    gnome-terminal --working-directory="$REPO_DIR" \
      -- bash -ic "source '$ROS_SETUP'; source '$WS_SETUP'; cd '$REPO_DIR'; $cmd; exec bash"
  else
    gnome-terminal --working-directory="$REPO_DIR" \
      -- bash -ic "source '$ROS_SETUP'; source '$WS_SETUP'; cd '$REPO_DIR'; echo 'Missing script: $script'; exec bash"
  fi
done

sleep 5

gnome-terminal \
  -- bash -ic "source '$ROS_SETUP'; source '$WS_SETUP'; cd '$ROS2_WS'; ros2 run my_drone_control outdoor_mission; exec bash"

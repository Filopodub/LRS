# my_drone_control

`takeoff_drone` performs a guarded ArduPilot/MAVROS takeoff:

1. Wait for `/mavros/state.connected`.
2. Request `GUIDED` mode and verify the response.
3. Arm and verify the response.
4. Request takeoff through `/mavros/cmd/takeoff`.
5. Wait for `/mavros/global_position/rel_alt` to reach the target altitude.

Build and run after MAVROS is running:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select my_drone_control
source install/setup.bash
ros2 run my_drone_control takeoff_drone
```

The node prompts for the target altitude in the terminal before it starts
waiting for MAVROS. Press Enter to use the default of 5 m, or enter a positive
value in meters. The default can also be changed without recompiling:

```bash
ros2 run my_drone_control takeoff_drone --ros-args -p target_altitude:=2.0
```

![alt text](miscellaneous/structure.png)

This node assumes MAVROS and the flight controller are already configured and
flight-ready. It does not land or disarm after reaching altitude.
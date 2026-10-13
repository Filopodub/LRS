# my_drone_control

By default, `takeoff_drone` reports the first pose received from
`/mavros/local_position/pose` as converted `x`, `y`, and `z` coordinates in
meters, then waits for a mission on `planned_commands` and executes it. To
report the position without flight actions, explicitly set `report_only` to
`true`:

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
ros2 run my_drone_control map_planner_node
```

In a second sourced terminal, start the takeoff node. It subscribes to the
planner's transient-local `planned_commands` mission, so it can start after the
planner has published it:

```bash
ros2 run my_drone_control takeoff_drone
```

`mission_demo_node` is an alternative mission source for testing. Do not run it
at the same time as `map_planner_node`, since both publish to `planned_commands`.

```bash
ros2 run my_drone_control takeoff_drone --ros-args -p report_only:=true
```

The default target is 2 m. Override it
without recompiling:

```bash
ros2 run my_drone_control takeoff_drone --ros-args -p target_altitude:=2.0
```

This node assumes MAVROS and the flight controller are already configured and
flight-ready. Test in SITL before using real hardware, and keep a manual abort
available. It does not land or disarm after reaching altitude.

whole structure were working on:
![alt text](/misselanious/image.png)
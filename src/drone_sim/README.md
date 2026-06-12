# drone_sim

`drone_sim` is a dependency-light Python ament package with ROS 2 quadrotor simulation samples. It targets Ubuntu 24.04/26.04 and ROS 2 Jazzy, Kilted, and Rolling using only common client libraries and message packages.

The simulator is intentionally kinematic rather than physics-heavy. It publishes `odom`, `pose`, `imu`, and TF for each drone, subscribes to `cmd_vel`, and can also chase `setpoint_pose` goals.

## Build

From the workspace root:

```bash
colcon build --packages-select drone_sim
source install/setup.bash
```

## Samples

### Single quad waypoint flight

```bash
ros2 launch drone_sim single_quad_waypoint.launch.py
```

This starts `sim_drone` and `waypoint_commander`. The commander cycles through waypoints from `config/single_quad.yaml` and publishes `setpoint_pose`.

### Altitude hold demo

```bash
ros2 launch drone_sim altitude_hold.launch.py target_altitude_m:=2.0
```

This starts `sim_drone` with `altitude_hold`, which publishes vertical `cmd_vel` commands to hold the configured altitude.

### Small namespaced swarm

```bash
ros2 launch drone_sim swarm.launch.py drone_count:=3 spacing_m:=2.0 altitude_m:=1.2
```

Each drone runs under `/drone_N` with its own `odom`, `pose`, `imu`, `cmd_vel`, and `setpoint_pose` topics. TF child frames are named `drone_N/base_link` to avoid frame collisions.

## Useful topics

```bash
ros2 topic echo /odom
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, z: 0.2}, angular: {z: 0.2}}"
ros2 topic pub /setpoint_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: odom}, pose: {position: {x: 1.0, y: 1.0, z: 1.5}, orientation: {w: 1.0}}}"
```

For a swarm drone, prefix topics with the namespace, for example `/drone_1/cmd_vel`.

## Files

- `drone_sim/sim_drone.py` - kinematic simulation node.
- `drone_sim/waypoint_commander.py` - repeating waypoint publisher.
- `drone_sim/altitude_hold.py` - simple altitude controller using `cmd_vel`.
- `launch/*.launch.py` - single drone, altitude hold, and swarm launch files.
- `config/*.yaml` - sample parameters.
- `urdf/quadrotor.urdf` and `rviz/drone_sim.rviz` - visualization helpers.

# drone_sim

> [!WARNING]
> 本パッケージは検証中であり、確実に動作確認したものではありません。詳細はリポジトリルートの README を参照してください。

`drone_sim` is a dependency-light Python ament package with ROS 2 quadrotor simulation samples. It targets Ubuntu 24.04/26.04 and ROS 2 Jazzy, Kilted, and Rolling using only common client libraries and message packages.

The simulator is intentionally kinematic rather than physics-heavy. It publishes `odom`, `pose`, `imu`, and TF for each drone, subscribes to `cmd_vel`, `wind_velocity`, geofence topics, and can also chase `setpoint_pose` goals.

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

### Wind, geofence, and telemetry demo

```bash
ros2 launch drone_sim wind_demo.launch.py
```

This starts `sim_drone`, `waypoint_commander`, `wind_disturbance`, `geofence_monitor`, and `telemetry_logger`. Wind is added to the kinematic integration, geofence breaches publish corrective `geofence_setpoint` messages, and telemetry summaries are published on `telemetry_summary`.

### Leader-follower formation demo

```bash
ros2 launch drone_sim formation_demo.launch.py
```

This starts a `/leader` drone with a waypoint commander and two followers. Each follower runs `formation_controller`, subscribes to `/leader/odom`, and publishes namespaced `setpoint_pose` commands with configured offsets. `config/formation.yaml` captures the same per-namespace parameters as the launch file's inline values (useful as a `--params-file` reference if you drive the nodes by hand instead of the launch file).

### Small namespaced swarm

```bash
ros2 launch drone_sim swarm.launch.py drone_count:=3 spacing_m:=2.0 altitude_m:=1.2
```

Each drone runs under `/drone_N` with its own `odom`, `pose`, `imu`, `cmd_vel`, and `setpoint_pose` topics. TF child frames are named `drone_N/base_link` to avoid frame collisions. `config/swarm.yaml` records the same `drone_count`/`spacing_m`/`altitude_m` values as this launch file's default arguments, but as plain top-level keys rather than a `ros__parameters` block for a real node name — it is not loadable via `--params-file` and is meant only as a human-readable reference for the launch argument defaults above.

### Noisy sensors demo

```bash
ros2 launch drone_sim noisy_sensors_demo.launch.py
```

This starts `sim_drone` and `waypoint_commander` using `config/noisy_sensors.yaml`. `sim_drone`
applies Gaussian noise to the published `odom` position and velocity and to the `imu` angular
velocity and linear acceleration, while the internal simulated state stays exact and TF and
`robot_status` continue to reflect the true pose. The noise is configured through `sim_drone`
parameters: `odom_position_noise_stddev`, `odom_velocity_noise_stddev`,
`imu_gyro_noise_stddev`, `imu_accel_noise_stddev`, and `imu_gyro_bias` (all default to `0.0`,
so noise is off unless configured). Setting `publish_ground_truth: true` additionally publishes
the true, noise-free pose on `ground_truth_pose` for comparison against the noisy `odom`/`pose`
topics. `drone_sim/noise_utils.py` implements the pure noise-generation helpers and is covered
by pytest unit tests, independent of ROS.

### Mission state machine demo

```bash
ros2 launch drone_sim mission_demo.launch.py
ros2 service call /start_mission std_srvs/srv/Trigger
ros2 topic echo /mission_state
ros2 service call /return_to_launch std_srvs/srv/Trigger
ros2 service call /abort_mission std_srvs/srv/Trigger
```

This starts `sim_drone`, `mission_state_machine`, `battery_monitor`, `emergency_land`, and
`geofence_monitor`. `mission_state_machine` walks the drone through
`IDLE -> TAKEOFF -> MISSION -> RTL -> LAND -> LANDED`: it stays in `IDLE` until
`/start_mission` is called (or `auto_start` is set), climbs to `takeoff_altitude_m`, then
cycles through the configured `waypoints`. It automatically switches to `RTL` once all
waypoints are visited or the battery drops to `rtl_battery_pct`, and returns home before
landing. Calling `/return_to_launch` requests an early RTL, while `/abort_mission` or a
critical-battery signal makes it land in place from any airborne state. The current state is
published on `/mission_state`. The pure transition rules are implemented in
`drone_sim/mission_logic.py` and covered by pytest unit tests, independent of ROS.

### Behavior-tree mission demo

```bash
ros2 launch drone_sim mission_bt_demo.launch.py
ros2 service call /start_mission std_srvs/srv/Trigger
ros2 topic echo /mission_state
ros2 topic echo /bt_trace
```

`mission_behavior_tree` runs the exact same mission as `mission_state_machine`
(same topics, services, and parameters, plus `publish_trace`) but drives it with a
behavior tree instead of an FSM. The minimal BT engine (Sequence / Selector /
Condition / Action) lives in `drone_sim/bt_core.py` and the mission tree in
`drone_sim/mission_bt.py`, both pure Python and pytest-covered. Each tick's path
through the tree is published on `/bt_trace`, so you can watch the emergency and
RTL branches preempt the mission branch in real time. See tutorial 11
(`docs/tutorials/11_behavior_tree.md`) for a side-by-side comparison of the two
implementations.

### Collision avoidance demo

```bash
ros2 launch drone_sim collision_avoidance_demo.launch.py
ros2 topic echo /drone_1/raw_setpoint_pose
ros2 topic echo /drone_1/setpoint_pose
```

This starts three namespaced drones (`/drone_1`, `/drone_2`, `/drone_3`). `/drone_2` and
`/drone_3` fly their own waypoint loops unmodified and act as moving traffic. `/drone_1`'s
`waypoint_commander` is remapped to publish on `raw_setpoint_pose` instead of `setpoint_pose`,
so the `collision_avoidance` node (loaded from `config/collision_avoidance.yaml`) can subscribe
to it, nudge the goal away from the other two drones using a potential-field repulsion, and
republish the adjusted goal on `setpoint_pose`, which `/drone_1`'s `sim_drone` then follows.
Compare `/drone_1/raw_setpoint_pose` (raw target) against `/drone_1/setpoint_pose` (the
adjusted goal `sim_drone` actually receives) to see the avoidance nudges in real time. Key parameters:
`drone_odom_topics` (the full swarm to react to), `own_odom_topic` (which drone this instance
protects), `safety_distance`/`influence_distance` (when repulsion starts and saturates), and
`repulsion_gain`/`max_adjustment` (strength and clamp of the correction). The pure force
computation lives in `drone_sim/collision_utils.py` and is covered by pytest unit tests,
independent of ROS; `drone_sim/collision_avoidance.py` is thin ROS wiring around it (odom/
setpoint subscriptions, a timer, and the adjusted-setpoint publisher) with no additional
standalone logic to unit test.

### Diagnostics publisher

```bash
ros2 run drone_sim diagnostics_publisher --ros-args -p robot_name:=drone_1 \
  -r odom:=/drone_1/odom -r battery:=/drone_1/battery
ros2 topic echo /diagnostics
```

`diagnostics_publisher` subscribes to `odom` and `battery` for a single drone and republishes a
standardized `diagnostic_msgs/DiagnosticArray` on `/diagnostics`, with one `DiagnosticStatus`
entry each for battery level, speed, and position. Status escalates from `OK` to `WARN`/`ERROR`
based on the `battery_warn_pct`/`battery_error_pct` and `speed_warn_ms` parameters, and reports
`STALE` until the first `odom`/`battery` message arrives.

## Dynamic parameters

`altitude_hold`, `sim_drone`, and `geofence_monitor` support runtime parameter updates via
`add_on_set_parameters_callback`; changes take effect on the next control/publish cycle
without restarting the node.

- `altitude_hold`: `kp`, `ki`, `kd`, and `target_altitude_m` can be changed live. Gains must be
  finite and `>= 0.0`; `target_altitude_m` must be finite and `>= 0.0`. Changing any of `kp`,
  `ki`, or `kd` resets the PID controller's accumulated integral and derivative state so the
  new gains don't act on stale error history.
- `sim_drone`: `max_linear_speed`, `max_yaw_rate`, `linear_accel_limit`, and
  `yaw_accel_limit` must be finite and `> 0.0`; `position_kp` and `yaw_kp` must be finite and
  `>= 0.0`; `cmd_timeout_sec` and `setpoint_timeout_sec` must be finite and `> 0.0`.
- `geofence_monitor`: `boundary_min_x`/`boundary_max_x`, `boundary_min_y`/`boundary_max_y`,
  `boundary_min_z`/`boundary_max_z`, and `margin_m` can be changed live. Boundaries are
  validated together, so the resulting `min < max` must hold on every axis, and `margin_m`
  must be finite and `>= 0.0`.

```bash
ros2 param set /altitude_hold kp 2.5
ros2 param set /sim_drone max_linear_speed 2.0
ros2 param set /geofence_monitor boundary_max_x 10.0
```

Invalid values (negative gains, non-finite numbers, or boundary changes that would make
`min >= max` on any axis) are rejected and the previous value is kept.

## Useful topics

```bash
ros2 topic echo /odom
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, z: 0.2}, angular: {z: 0.2}}"
ros2 topic pub /setpoint_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: odom}, pose: {position: {x: 1.0, y: 1.0, z: 1.5}, orientation: {w: 1.0}}}"
ros2 topic echo /wind_velocity
ros2 topic echo /geofence_breach
ros2 topic echo /telemetry_summary
```

For a swarm drone, prefix topics with the namespace, for example `/drone_1/cmd_vel`.

## Files

- `drone_sim/sim_drone.py` - kinematic simulation node.
- `drone_sim/waypoint_commander.py` - repeating waypoint publisher.
- `drone_sim/altitude_hold.py` - simple altitude controller using `cmd_vel`.
- `drone_sim/wind_disturbance.py` - time-varying wind vector publisher.
- `drone_sim/geofence_monitor.py` - bounding-box monitor and corrective setpoint publisher.
- `drone_sim/formation_controller.py` - leader-follower setpoint generator.
- `drone_sim/telemetry_logger.py` - flight statistics logger and summary publisher.
- `drone_sim/battery_monitor.py` and `drone_sim/emergency_land.py` - battery drain and landing helpers.
- `drone_sim/mission_logic.py` and `drone_sim/mission_state_machine.py` - pure mission
  transition rules and the ROS node that drives takeoff/mission/RTL/land.
- `drone_sim/bt_core.py` and `drone_sim/mission_bt.py` - minimal behavior-tree engine and the
  mission tree built on it.
- `drone_sim/mission_behavior_tree.py` - the ROS node that drives the mission using the
  behavior tree instead of the FSM.
- `drone_sim/collision_utils.py` and `drone_sim/collision_avoidance.py` - pure potential-field
  repulsion math and the ROS node that applies it to a drone's setpoint.
- `drone_sim/diagnostics_publisher.py` - aggregates odom/battery into a standardized
  `diagnostic_msgs/DiagnosticArray`.
- `launch/*.launch.py` - single drone, altitude hold, battery, mission, mission behavior tree,
  wind/geofence/telemetry, formation, swarm, and collision avoidance launch files.
- `config/*.yaml` - sample parameters.
- `urdf/quadrotor.urdf` and `rviz/drone_sim.rviz` - visualization helpers.

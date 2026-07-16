# ground_robot_sim

`ground_robot_sim` is a dependency-light ROS 2 Python/ament sample package for a planar ground robot. It targets Ubuntu 24.04/26.04 with ROS 2 Jazzy, Kilted, or Rolling.

The simulator node accepts `cmd_vel` and publishes:

- `odom` (`nav_msgs/Odometry`)
- `tf` transforms from `odom` to `base_link` and from `base_link` to `base_scan`
- `scan` (`sensor_msgs/LaserScan`) from a simple 180-degree synthetic lidar

## Build

From the workspace root:

```bash
colcon build --packages-select ground_robot_sim
source install/setup.bash
```

## Samples

Run an open-loop square patrol:

```bash
ros2 launch ground_robot_sim diff_drive_patrol.launch.py
```

Run a lidar obstacle stop demo. The robot moves forward until the front scan sector sees an obstacle inside the stop distance:

```bash
ros2 launch ground_robot_sim lidar_obstacle_stop.launch.py
```

Run a closed-loop waypoint follower. The robot drives through a configurable list of (x, y) waypoints and loops indefinitely:

```bash
ros2 launch ground_robot_sim waypoint_follower.launch.py
# または個別ノードを起動する場合
ros2 run ground_robot_sim ground_robot_node
ros2 run ground_robot_sim waypoint_follower
```

Run a lidar obstacle avoidance demo. The robot steers away from obstacles instead of only stopping — forward speed is scaled down as obstacles approach and the robot rotates away from the nearer side:

```bash
ros2 launch ground_robot_sim lidar_obstacle_avoid.launch.py
# または個別ノードを起動する場合
ros2 run ground_robot_sim ground_robot_node
ros2 run ground_robot_sim lidar_obstacle_avoid
```

Run a noisy-sensors demo. Odometry and lidar readings are perturbed with Gaussian noise while
the internal state and TF stay exact, and the true pose is published on `ground_truth_pose` for
comparison:

```bash
ros2 launch ground_robot_sim noisy_sensors_demo.launch.py
```

Run three robots in separate namespaces. Topics are under `/robot1`, `/robot2`, and `/robot3`; TF frames are prefixed the same way:

```bash
ros2 launch ground_robot_sim multi_robot.launch.py
```

Run the optional GZ Sim integration demo. This starts GZ Sim, spawns the URDF model, bridges `/cmd_vel`, `/odom`, `/tf`, and `/joint_states`, and runs the patrol controller:

```bash
ros2 launch ground_robot_sim gazebo.launch.py use_gui:=false
```

For manual command testing, disable the patrol controller:

```bash
ros2 launch ground_robot_sim gazebo.launch.py use_gui:=true start_controller:=false
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.4}}" --rate 10
```

Inspect topics:

```bash
ros2 topic echo /odom
ros2 topic echo /scan
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.3}}"
```

For a namespaced robot, use namespaced topics:

```bash
ros2 topic echo /robot1/odom
ros2 topic pub /robot1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.3}}"
```

### Keyboard teleop

Drive a robot (or a drone) interactively from the terminal:

```bash
ros2 run ground_robot_sim teleop_keyboard
```

| Key | Action |
| --- | --- |
| `w` / `s` | drive forward / backward |
| `a` / `d` | turn left / right |
| `r` / `f` | climb / descend (`linear.z`, useful for drones) |
| `q` / `z` | increase / decrease speed scale |
| `space` | stop immediately |

The node also stops automatically (`key_timeout_sec`, default `0.5`) if no key is
pressed for a while, so a dropped terminal or a stuck key never leaves the robot
running unattended.

`teleop_keyboard` publishes plain `cmd_vel`, so remap it to drive a specific robot
or drone in a multi-robot demo:

```bash
ros2 run ground_robot_sim teleop_keyboard --ros-args -r cmd_vel:=/drone_1/cmd_vel
ros2 run ground_robot_sim teleop_keyboard --ros-args -r cmd_vel:=/robot_1/cmd_vel
```

## Useful parameters

- `initial_x`, `initial_y`, `initial_yaw`: starting pose in the square world.
- `frame_prefix`: TF frame prefix used by multi-robot launches.
- `obstacles`: flat list of circular obstacles as `[x, y, radius, ...]`.
- `world_half_size`: half-width of the square simulated room.
- `scan_samples`, `scan_range_min`, `scan_range_max`: synthetic lidar behavior.
- `odom_position_noise_stddev`, `odom_yaw_noise_stddev`, `odom_velocity_noise_stddev`: Gaussian
  noise stddevs applied to the published `odom` pose and twist. Default `0.0` (no noise). The
  internal true pose and the `tf` broadcast are never affected.
- `scan_range_noise_stddev`: Gaussian noise stddev applied to each published `scan` range
  reading, clamped to `[scan_range_min, scan_range_max]`. Default `0.0` (no noise). Non-finite
  readings (out-of-range rays) are left unchanged.
- `publish_ground_truth`: when `true`, also publish the exact pose on `ground_truth_pose`
  (`geometry_msgs/PoseStamped`) at the odom publish rate, for comparing against the noisy
  `odom` topic. Default `false`.

### waypoint_follower ノードのパラメータ

- `waypoints`: 目標地点の平坦リスト `[x1, y1, x2, y2, ...]`。デフォルトは 1.5 m 四方の正方形ルート。
- `tolerance_m`: 到着判定距離（メートル）。デフォルト `0.15`。
- `hold_time_sec`: 各ウェイポイントで停止する時間（秒）。デフォルト `0.5`。
- `loop`: ウェイポイント列を繰り返すか否か。デフォルト `true`。
- `max_linear_speed` / `max_angular_speed`: 速度上限。デフォルト `0.4` / `1.2`。
- `kp_linear` / `kp_angular`: 比例ゲイン。デフォルト `0.8` / `1.5`。
- `heading_gate_rad`: この角度誤差以下のときのみ前進する閾値。デフォルト `0.5`。

### lidar_obstacle_avoid ノードのパラメータ

- `forward_speed`: 障害物がない場合の前進速度（m/s）。デフォルト `0.25`。
- `avoid_distance`: この距離以下で回避行動を開始する距離（メートル）。デフォルト `1.2`。
- `stop_distance`: この距離以下で完全停止し回転する距離（メートル）。デフォルト `0.45`。
- `turn_speed`: 回避時の旋回角速度（rad/s）。デフォルト `0.8`。
- `front_angle_degrees`: 前方センサー扇形の全角（度）。デフォルト `70.0`。

## Dynamic parameters

`waypoint_follower` and `lidar_obstacle_avoid` support runtime parameter updates via
`add_on_set_parameters_callback`, so gains and thresholds can be tuned while the demo runs.

- `waypoint_follower`: `kp_linear` / `ki_linear` / `kd_linear`, `kp_angular` / `ki_angular` /
  `kd_angular` (finite, `>= 0.0`) and `tolerance_m` (finite, `> 0.0`). Changing any gain resets
  that axis's PID integral/derivative state so the controller doesn't jump on the next tick.
- `lidar_obstacle_avoid`: `stop_distance` and `avoid_distance` (finite, `> 0.0`).
  `stop_distance` must always remain strictly less than `avoid_distance`; an update that would
  violate this is rejected and the previous values are kept.

```bash
ros2 param set /waypoint_follower kp_linear 1.0
ros2 param set /waypoint_follower tolerance_m 0.1
ros2 param set /lidar_obstacle_avoid stop_distance 0.6
ros2 param set /lidar_obstacle_avoid avoid_distance 1.5
```

Most samples intentionally avoid physics engine dependencies so they can run quickly in teaching, CI, and container environments. The `gazebo.launch.py` demo is an optional GZ Sim path for learning URDF/SDF, Gazebo plugins, and `ros_gz_bridge`.

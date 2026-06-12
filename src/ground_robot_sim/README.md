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

Run three robots in separate namespaces. Topics are under `/robot1`, `/robot2`, and `/robot3`; TF frames are prefixed the same way:

```bash
ros2 launch ground_robot_sim multi_robot.launch.py
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

## Useful parameters

- `initial_x`, `initial_y`, `initial_yaw`: starting pose in the square world.
- `frame_prefix`: TF frame prefix used by multi-robot launches.
- `obstacles`: flat list of circular obstacles as `[x, y, radius, ...]`.
- `world_half_size`: half-width of the square simulated room.
- `scan_samples`, `scan_range_min`, `scan_range_max`: synthetic lidar behavior.

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

The code intentionally avoids physics engine and Gazebo dependencies so it can run quickly in teaching, CI, and container environments.

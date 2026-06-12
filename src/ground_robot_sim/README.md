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

The code intentionally avoids physics engine and Gazebo dependencies so it can run quickly in teaching, CI, and container environments.

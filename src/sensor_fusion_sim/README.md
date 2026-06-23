# sensor_fusion_sim

`sensor_fusion_sim` is a dependency-light ROS 2 Python/ament sample package for learning sensor fusion, lifecycle nodes, QoS profiles, and callback groups.

The package contains three nodes:

- `noisy_sensor_node` — publishes noisy GPS (`PointStamped`, RELIABLE, 1 Hz), IMU (`Imu`, BEST_EFFORT, 50 Hz), wheel odometry (`Odometry`, RELIABLE, 10 Hz), and ground truth along a circular trajectory.
- `complementary_filter` — fuses GPS, IMU, and wheel odometry using a complementary filter and publishes `fused_odom` (`Odometry`) and `filter_diagnostics` (`String`). Uses `ReentrantCallbackGroup` for sensor callbacks and `MutuallyExclusiveCallbackGroup` for publish timers with a `MultiThreadedExecutor`.
- `lifecycle_data_recorder` — a `LifecycleNode` that records `fused_odom` into a bounded buffer while in the active state and publishes recording status and summary.

## Build

From the workspace root:

```bash
colcon build --packages-select sensor_fusion_sim
source install/setup.bash
```

## Sample

Run the sensor fusion demo:

```bash
ros2 launch sensor_fusion_sim sensor_fusion_demo.launch.py
```

This starts all three nodes. The lifecycle data recorder is automatically configured and activated by the launch file.

Inspect topics:

```bash
ros2 topic echo /fused_odom
ros2 topic echo /ground_truth
ros2 topic echo /gps
ros2 topic echo /imu
ros2 topic echo /filter_diagnostics
ros2 topic echo /recording_status
```

Dynamically tune filter weights at runtime:

```bash
ros2 param set /complementary_filter gps_alpha 0.3
ros2 param set /complementary_filter odom_alpha 0.5
ros2 param set /complementary_filter imu_yaw_weight 0.1
```

## Useful parameters

### noisy_sensor_node

- `circle_radius`: circular trajectory radius in meters (default `5.0`).
- `circle_omega`: angular velocity of the circular trajectory in rad/s (default `0.3`).
- `gps_noise_stddev`: GPS position noise standard deviation in meters (default `0.5`).
- `imu_accel_stddev` / `imu_gyro_stddev`: IMU noise standard deviations (default `0.1` / `0.02`).
- `odom_noise_stddev`: wheel odometry noise standard deviation (default `0.05`).

### complementary_filter

- `gps_alpha`: GPS position blending weight (default `0.15`).
- `odom_alpha`: wheel odometry position blending weight (default `0.30`).
- `imu_yaw_weight`: IMU yaw blending weight (default `0.05`).
- `publish_rate_hz`: fused output publish rate (default `20.0`).

### lifecycle_data_recorder

- `max_buffer_size`: maximum number of records to keep (default `500`).
- `publish_rate_hz`: status publish rate (default `1.0`).
- `input_topic`: input topic name (default `fused_odom`).

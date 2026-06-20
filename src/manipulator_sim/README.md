# manipulator_sim

`manipulator_sim` is a dependency-light ROS 2 Python/ament sample package for a 2-DOF planar manipulator.

The simulator publishes:

- `joint_states` (`sensor_msgs/JointState`)
- `tool_pose` (`geometry_msgs/PoseStamped`)
- TF transforms (`base_link -> link1 -> tool0`)

The commander publishes `joint_target` (`sensor_msgs/JointState`) from planar `(x, y)` targets.

## Build

From the workspace root:

```bash
colcon build --packages-select manipulator_sim
source install/setup.bash
```

## Sample

Run a planar reach demo:

```bash
ros2 launch manipulator_sim planar_reach_demo.launch.py
```

This starts `manipulator_simulator` and `target_commander`. The commander cycles through reachable targets in `config/planar_reach_demo.yaml` and sends joint targets.

Inspect topics:

```bash
ros2 topic echo /joint_states
ros2 topic echo /tool_pose
ros2 topic echo /joint_target
```

## Useful parameters

- `link_lengths`: 2-link arm length list `[l1, l2]`.
- `max_joint_speed`: per-joint speed limit in rad/s.
- `targets_xy`: flat target list `[x1, y1, x2, y2, ...]`.
- `hold_time_sec`: dwell time after reaching each target.
- `elbow_up`: choose inverse-kinematics branch.

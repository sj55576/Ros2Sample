# Status information for a simulated robot.

std_msgs/Header header

# Identifying name of the robot (e.g. "drone_0", "robot1").
string robot_name

# Current operating state: "idle", "moving", "emergency_stop", "landing", "landed".
string state

# Battery charge remaining as a percentage (0.0 to 100.0).
float64 battery_percentage

# Current 3-D position in the odom frame.
geometry_msgs/Point position

# Current linear velocity vector in the body frame.
geometry_msgs/Vector3 linear_velocity

# Current heading angle in radians [-pi, pi].
float64 heading_rad

"""A dependency-light kinematic quadrotor simulator node."""

from math import atan2, hypot
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sample_interfaces.msg import RobotStatus
from sample_interfaces.srv import GetRobotStatus
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster

from drone_sim.math_utils import clamp, normalize_angle, quat_from_euler


class SimDrone(Node):
    """Integrate a simple drone model and publish common ROS state topics."""

    def __init__(self) -> None:
        super().__init__('sim_drone')

        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('mass_kg', 1.2)
        self.declare_parameter('linear_accel_limit', 3.0)
        self.declare_parameter('yaw_accel_limit', 4.0)
        self.declare_parameter('max_linear_speed', 5.0)
        self.declare_parameter('max_yaw_rate', 2.5)
        self.declare_parameter('cmd_timeout_sec', 0.6)
        self.declare_parameter('setpoint_timeout_sec', 1.0)
        self.declare_parameter('position_kp', 1.2)
        self.declare_parameter('yaw_kp', 1.8)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_z', 0.0)
        self.declare_parameter('initial_yaw', 0.0)

        self.frame_id = self.get_parameter('frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.linear_accel_limit = float(self.get_parameter('linear_accel_limit').value)
        self.yaw_accel_limit = float(self.get_parameter('yaw_accel_limit').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_yaw_rate = float(self.get_parameter('max_yaw_rate').value)
        self.cmd_timeout_sec = float(self.get_parameter('cmd_timeout_sec').value)
        self.setpoint_timeout_sec = float(self.get_parameter('setpoint_timeout_sec').value)
        self.position_kp = float(self.get_parameter('position_kp').value)
        self.yaw_kp = float(self.get_parameter('yaw_kp').value)

        self.x = float(self.get_parameter('initial_x').value)
        self.y = float(self.get_parameter('initial_y').value)
        self.z = float(self.get_parameter('initial_z').value)
        self.yaw = float(self.get_parameter('initial_yaw').value)
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.yaw_rate = 0.0
        self.last_ax = 0.0
        self.last_ay = 0.0
        self.last_az = 0.0

        self.cmd_vel = Twist()
        self.last_cmd_time = self.get_clock().now()
        self.setpoint: Optional[PoseStamped] = None
        self.last_setpoint_time = self.get_clock().now()
        self.last_update_time = self.get_clock().now()
        self.wind_x = 0.0
        self.wind_y = 0.0
        self.wind_z = 0.0
        self._geofence_breached = False

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.pose_pub = self.create_publisher(PoseStamped, 'pose', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu', 10)
        self.create_subscription(Twist, 'cmd_vel', self._on_cmd_vel, 10)
        self.create_subscription(PoseStamped, 'setpoint_pose', self._on_setpoint_pose, 10)
        self.create_subscription(Vector3, 'wind_velocity', self._on_wind, 10)
        self.create_subscription(Bool, 'geofence_breach', self._on_geofence_breach, 10)
        self.create_subscription(
            PoseStamped, 'geofence_setpoint', self._on_geofence_setpoint, 10
        )
        self._battery_pct: float = 100.0
        self.status_pub = self.create_publisher(RobotStatus, 'robot_status', 10)
        self.create_subscription(BatteryState, 'battery', self._on_battery, 10)
        self.create_service(
            GetRobotStatus, 'get_robot_status', self._get_status_callback
        )
        self.tf_broadcaster = TransformBroadcaster(self)

        period = 1.0 / max(self.publish_rate_hz, 1.0)
        self.create_timer(period, self._step)
        self.get_logger().info(
            f'Simulating {self.get_fully_qualified_name()} in frame {self.frame_id}'
        )

    def _on_cmd_vel(self, msg: Twist) -> None:
        self.cmd_vel = msg
        self.last_cmd_time = self.get_clock().now()

    def _on_setpoint_pose(self, msg: PoseStamped) -> None:
        self.setpoint = msg
        self.last_setpoint_time = self.get_clock().now()

    def _on_wind(self, msg: Vector3) -> None:
        self.wind_x = msg.x
        self.wind_y = msg.y
        self.wind_z = msg.z

    def _on_geofence_breach(self, msg: Bool) -> None:
        self._geofence_breached = msg.data

    def _on_geofence_setpoint(self, msg: PoseStamped) -> None:
        if self._geofence_breached:
            self.setpoint = msg
            self.last_setpoint_time = self.get_clock().now()

    def _on_battery(self, msg: BatteryState) -> None:
        """Track the latest battery percentage."""
        self._battery_pct = msg.percentage * 100.0

    def _step(self) -> None:
        now = self.get_clock().now()
        dt = max((now - self.last_update_time).nanoseconds * 1e-9, 1e-4)
        self.last_update_time = now

        desired = self._desired_twist(now)
        desired_vx = clamp(desired.linear.x, -self.max_linear_speed, self.max_linear_speed)
        desired_vy = clamp(desired.linear.y, -self.max_linear_speed, self.max_linear_speed)
        desired_vz = clamp(desired.linear.z, -self.max_linear_speed, self.max_linear_speed)
        desired_yaw_rate = clamp(desired.angular.z, -self.max_yaw_rate, self.max_yaw_rate)

        self.last_ax = self._approach_velocity('vx', desired_vx, self.linear_accel_limit, dt)
        self.last_ay = self._approach_velocity('vy', desired_vy, self.linear_accel_limit, dt)
        self.last_az = self._approach_velocity('vz', desired_vz, self.linear_accel_limit, dt)
        self._approach_velocity('yaw_rate', desired_yaw_rate, self.yaw_accel_limit, dt)

        self.x += (self.vx + self.wind_x) * dt
        self.y += (self.vy + self.wind_y) * dt
        self.z = max(0.0, (self.z + (self.vz + self.wind_z) * dt))
        self.yaw = normalize_angle(self.yaw + self.yaw_rate * dt)

        self._publish_state(now)

    def _desired_twist(self, now) -> Twist:
        setpoint_age = (now - self.last_setpoint_time).nanoseconds * 1e-9
        if self.setpoint is not None and setpoint_age <= self.setpoint_timeout_sec:
            desired = Twist()
            dx = self.setpoint.pose.position.x - self.x
            dy = self.setpoint.pose.position.y - self.y
            dz = self.setpoint.pose.position.z - self.z
            speed_limit = self.max_linear_speed
            desired.linear.x = clamp(dx * self.position_kp, -speed_limit, speed_limit)
            desired.linear.y = clamp(dy * self.position_kp, -speed_limit, speed_limit)
            desired.linear.z = clamp(dz * self.position_kp, -speed_limit, speed_limit)
            heading_error = normalize_angle(atan2(dy, dx) - self.yaw)
            desired.angular.z = clamp(
                heading_error * self.yaw_kp, -self.max_yaw_rate, self.max_yaw_rate
            )
            if hypot(dx, dy) < 0.05:
                desired.angular.z = 0.0
            return desired

        cmd_age = (now - self.last_cmd_time).nanoseconds * 1e-9
        if cmd_age <= self.cmd_timeout_sec:
            return self.cmd_vel
        return Twist()

    def _approach_velocity(self, attr: str, target: float, accel_limit: float, dt: float) -> float:
        current = getattr(self, attr)
        delta = clamp(target - current, -accel_limit * dt, accel_limit * dt)
        setattr(self, attr, current + delta)
        return delta / dt

    def _build_status(self) -> RobotStatus:
        """Build the current robot status message."""
        speed = (self.vx ** 2 + self.vy ** 2 + self.vz ** 2) ** 0.5
        if speed > 0.05:
            state = 'moving'
        else:
            state = 'idle'
        status = RobotStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.header.frame_id = self.frame_id
        status.robot_name = self.get_fully_qualified_name()
        status.state = state
        status.battery_percentage = self._battery_pct
        status.position.x = self.x
        status.position.y = self.y
        status.position.z = self.z
        status.linear_velocity.x = self.vx
        status.linear_velocity.y = self.vy
        status.linear_velocity.z = self.vz
        status.heading_rad = self.yaw
        return status

    def _get_status_callback(
        self,
        request: GetRobotStatus.Request,
        response: GetRobotStatus.Response,
    ) -> GetRobotStatus.Response:
        """Return the current robot status."""
        response.status = self._build_status()
        response.success = True
        response.message = 'OK'
        return response

    def _publish_state(self, stamp) -> None:
        quat = quat_from_euler(0.0, 0.0, self.yaw)

        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = self.z
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.linear.z = self.vz
        odom.twist.twist.angular.z = self.yaw_rate
        self.odom_pub.publish(odom)

        pose = PoseStamped()
        pose.header = odom.header
        pose.pose = odom.pose.pose
        self.pose_pub.publish(pose)

        imu = Imu()
        imu.header.stamp = odom.header.stamp
        imu.header.frame_id = self.base_frame_id
        imu.orientation = odom.pose.pose.orientation
        imu.angular_velocity.z = self.yaw_rate
        imu.linear_acceleration = Vector3(x=self.last_ax, y=self.last_ay, z=self.last_az + 9.80665)
        self.imu_pub.publish(imu)

        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = self.base_frame_id
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = self.z
        transform.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)
        self.status_pub.publish(self._build_status())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimDrone()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

"""A tiny differential-drive robot simulator with odometry, TF, and fake lidar."""

import math

from geometry_msgs.msg import TransformStamped, Twist
from ground_robot_sim.geometry import normalize_angle, parse_circles
from ground_robot_sim.geometry import ray_circle_distance, yaw_to_quaternion
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sample_interfaces.msg import RobotStatus
from sample_interfaces.srv import GetRobotStatus
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster


class GroundRobotNode(Node):
    """Simulate a planar robot that listens to cmd_vel and publishes common topics."""

    def __init__(self) -> None:
        super().__init__('ground_robot')
        self.declare_parameter('robot_name', '')
        self.declare_parameter('frame_prefix', '')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('laser_frame', 'base_scan')
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('scan_rate', 10.0)
        self.declare_parameter('wheel_base', 0.36)
        self.declare_parameter('max_linear_speed', 0.8)
        self.declare_parameter('max_angular_speed', 1.8)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)
        self.declare_parameter('scan_range_min', 0.08)
        self.declare_parameter('scan_range_max', 8.0)
        self.declare_parameter('scan_samples', 181)
        self.declare_parameter('world_half_size', 5.0)
        self.declare_parameter('obstacles', [2.0, 0.0, 0.45, -1.25, 1.2, 0.35, 0.5, -2.0, 0.5])

        self.robot_name = self.get_parameter('robot_name').value
        frame_prefix = str(self.get_parameter('frame_prefix').value)
        if frame_prefix and not frame_prefix.endswith('/'):
            frame_prefix += '/'
        self.odom_frame = frame_prefix + str(self.get_parameter('odom_frame').value)
        self.base_frame = frame_prefix + str(self.get_parameter('base_frame').value)
        self.laser_frame = frame_prefix + str(self.get_parameter('laser_frame').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.range_min = float(self.get_parameter('scan_range_min').value)
        self.range_max = float(self.get_parameter('scan_range_max').value)
        self.scan_samples = max(3, int(self.get_parameter('scan_samples').value))
        self.world_half_size = float(self.get_parameter('world_half_size').value)
        self.obstacles = parse_circles(self.get_parameter('obstacles').value)

        self.x = float(self.get_parameter('initial_x').value)
        self.y = float(self.get_parameter('initial_y').value)
        self.yaw = float(self.get_parameter('initial_yaw').value)
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.last_update_time = self.get_clock().now()
        self._emergency_stopped = False

        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.scan_publisher = self.create_publisher(LaserScan, 'scan', 10)
        self.status_pub = self.create_publisher(RobotStatus, 'robot_status', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.create_service(Trigger, 'emergency_stop', self._emergency_stop_callback)
        self.create_service(Trigger, 'reset_emergency', self._reset_emergency_callback)
        self.create_service(GetRobotStatus, 'get_robot_status', self._get_status_callback)

        publish_rate = max(1.0, float(self.get_parameter('publish_rate').value))
        scan_rate = max(1.0, float(self.get_parameter('scan_rate').value))
        self.create_timer(1.0 / publish_rate, self.update_state)
        self.create_timer(1.0 / scan_rate, self.publish_scan)
        self.get_logger().info(
            f"Started ground robot sim '{self.robot_name or self.get_namespace()}' "
            f'with base frame {self.base_frame}'
        )

    def _emergency_stop_callback(
        self, request: Trigger.Request, response: Trigger.Response,
    ) -> Trigger.Response:
        """Activate emergency stop - zero all velocities and reject cmd_vel."""
        self._emergency_stopped = True
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        response.success = True
        response.message = 'Emergency stop activated'
        self.get_logger().warn('Emergency stop activated!')
        return response

    def _reset_emergency_callback(
        self, request: Trigger.Request, response: Trigger.Response,
    ) -> Trigger.Response:
        """Deactivate emergency stop - resume accepting cmd_vel."""
        self._emergency_stopped = False
        response.success = True
        response.message = 'Emergency stop released'
        self.get_logger().info('Emergency stop released')
        return response

    def cmd_vel_callback(self, msg: Twist) -> None:
        """Store a bounded velocity command for the next integration tick."""
        if self._emergency_stopped:
            return
        self.linear_velocity = max(
            -self.max_linear_speed,
            min(self.max_linear_speed, msg.linear.x),
        )
        self.angular_velocity = max(
            -self.max_angular_speed,
            min(self.max_angular_speed, msg.angular.z),
        )

    def _build_status(self) -> RobotStatus:
        status = RobotStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.header.frame_id = self.odom_frame
        status.robot_name = self.robot_name or self.get_fully_qualified_name()
        if self._emergency_stopped:
            status.state = 'emergency_stop'
        elif abs(self.linear_velocity) > 0.01 or abs(self.angular_velocity) > 0.01:
            status.state = 'moving'
        else:
            status.state = 'idle'
        status.battery_percentage = 100.0
        status.position.x = self.x
        status.position.y = self.y
        status.position.z = 0.0
        status.linear_velocity.x = self.linear_velocity
        status.linear_velocity.y = 0.0
        status.linear_velocity.z = 0.0
        status.heading_rad = self.yaw
        return status

    def _get_status_callback(
        self,
        request: GetRobotStatus.Request,
        response: GetRobotStatus.Response,
    ) -> GetRobotStatus.Response:
        response.status = self._build_status()
        response.success = True
        response.message = 'OK'
        return response

    def update_state(self) -> None:
        """Integrate diff-drive motion and publish odometry plus odom->base TF."""
        now = self.get_clock().now()
        dt = max(0.0, (now - self.last_update_time).nanoseconds * 1e-9)
        self.last_update_time = now

        self.x += self.linear_velocity * math.cos(self.yaw) * dt
        self.y += self.linear_velocity * math.sin(self.yaw) * dt
        self.yaw = normalize_angle(self.yaw + self.angular_velocity * dt)

        quat_x, quat_y, quat_z, quat_w = yaw_to_quaternion(self.yaw)
        stamp = now.to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = quat_x
        odom.pose.pose.orientation.y = quat_y
        odom.pose.pose.orientation.z = quat_z
        odom.pose.pose.orientation.w = quat_w
        odom.twist.twist.linear.x = self.linear_velocity
        odom.twist.twist.angular.z = self.angular_velocity
        self.odom_publisher.publish(odom)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation.x = quat_x
        transform.transform.rotation.y = quat_y
        transform.transform.rotation.z = quat_z
        transform.transform.rotation.w = quat_w
        self.tf_broadcaster.sendTransform(transform)

        laser_transform = TransformStamped()
        laser_transform.header.stamp = stamp
        laser_transform.header.frame_id = self.base_frame
        laser_transform.child_frame_id = self.laser_frame
        laser_transform.transform.translation.x = 0.18
        laser_transform.transform.translation.z = 0.12
        laser_transform.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(laser_transform)
        self.status_pub.publish(self._build_status())

    def publish_scan(self) -> None:
        """Publish a simple 180-degree scan against circular obstacles and world walls."""
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.laser_frame
        scan.angle_min = -math.pi / 2.0
        scan.angle_max = math.pi / 2.0
        scan.angle_increment = (scan.angle_max - scan.angle_min) / float(self.scan_samples - 1)
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / max(1.0, float(self.get_parameter('scan_rate').value))
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = [
            self.cast_ray(scan.angle_min + i * scan.angle_increment)
            for i in range(self.scan_samples)
        ]
        self.scan_publisher.publish(scan)

    def cast_ray(self, relative_angle: float) -> float:
        """Cast one ray and return the nearest range reading."""
        angle = self.yaw + relative_angle
        ray_x = math.cos(angle)
        ray_y = math.sin(angle)
        distance = self.distance_to_world_wall(self.x, self.y, ray_x, ray_y)
        for obstacle in self.obstacles:
            distance = min(distance, ray_circle_distance(self.x, self.y, ray_x, ray_y, obstacle))
        if not math.isfinite(distance) or distance > self.range_max:
            return self.range_max
        return max(self.range_min, distance)

    def distance_to_world_wall(
        self,
        origin_x: float,
        origin_y: float,
        ray_x: float,
        ray_y: float,
    ) -> float:
        """Return the distance from a ray to the square world's boundary."""
        candidates = []
        if abs(ray_x) > 1e-9:
            candidates.extend([
                (self.world_half_size - origin_x) / ray_x,
                (-self.world_half_size - origin_x) / ray_x,
            ])
        if abs(ray_y) > 1e-9:
            candidates.extend([
                (self.world_half_size - origin_y) / ray_y,
                (-self.world_half_size - origin_y) / ray_y,
            ])
        distances = [candidate for candidate in candidates if candidate >= 0.0]
        return min(distances) if distances else math.inf


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GroundRobotNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

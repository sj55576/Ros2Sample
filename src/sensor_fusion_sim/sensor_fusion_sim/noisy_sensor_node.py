"""Simulated platform moving in a circle, publishing noisy sensor data."""

import math
import random

import rclpy
from geometry_msgs.msg import PointStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster

from sensor_fusion_sim.noise_model import (
    add_gaussian_noise_3d,
    drift_walk,
    generate_imu_noise,
)


def _quat_from_yaw(yaw: float):
    """Return (x, y, z, w) quaternion for a pure yaw rotation."""
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class NoisySensorNode(Node):
    """Publish noisy GPS, IMU, and wheel odometry for a circular trajectory."""

    def __init__(self) -> None:
        super().__init__('noisy_sensor_node')

        self.declare_parameter('circle_radius', 5.0)
        self.declare_parameter('circle_omega', 0.3)
        self.declare_parameter('gps_rate_hz', 1.0)
        self.declare_parameter('gps_noise_stddev', 0.5)
        self.declare_parameter('imu_rate_hz', 50.0)
        self.declare_parameter('imu_accel_stddev', 0.1)
        self.declare_parameter('imu_gyro_stddev', 0.02)
        self.declare_parameter('odom_rate_hz', 10.0)
        self.declare_parameter('odom_noise_stddev', 0.05)
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('base_frame_id', 'base_link')

        self._radius = float(
            self.get_parameter('circle_radius').value
        )
        self._omega = float(
            self.get_parameter('circle_omega').value
        )
        self._gps_rate = float(
            self.get_parameter('gps_rate_hz').value
        )
        self._gps_stddev = float(
            self.get_parameter('gps_noise_stddev').value
        )
        self._imu_rate = float(
            self.get_parameter('imu_rate_hz').value
        )
        self._accel_stddev = float(
            self.get_parameter('imu_accel_stddev').value
        )
        self._gyro_stddev = float(
            self.get_parameter('imu_gyro_stddev').value
        )
        self._odom_rate = float(
            self.get_parameter('odom_rate_hz').value
        )
        self._odom_stddev = float(
            self.get_parameter('odom_noise_stddev').value
        )
        self._frame_id = str(
            self.get_parameter('frame_id').value
        )
        self._base_frame_id = str(
            self.get_parameter('base_frame_id').value
        )

        self._accel_bias = 0.0
        self._gyro_bias = 0.0

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._gps_pub = self.create_publisher(
            PointStamped, 'gps', reliable_qos
        )
        self._imu_pub = self.create_publisher(
            Imu, 'imu', best_effort_qos
        )
        self._odom_pub = self.create_publisher(
            Odometry, 'wheel_odom', reliable_qos
        )
        self._gt_pub = self.create_publisher(
            Odometry, 'ground_truth', reliable_qos
        )

        self._tf_broadcaster = TransformBroadcaster(self)

        self.create_timer(
            1.0 / max(self._gps_rate, 0.1), self._publish_gps
        )
        self.create_timer(
            1.0 / max(self._imu_rate, 1.0), self._publish_imu
        )
        self.create_timer(
            1.0 / max(self._odom_rate, 1.0), self._publish_odom
        )

        self.get_logger().info(
            f'NoisySensorNode started: R={self._radius} m, '
            f'omega={self._omega} rad/s'
        )

    def _elapsed(self) -> float:
        """Return seconds since node start."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _true_state(self, t: float):
        """Return (x, y, yaw, vx, vy, yaw_rate) for circular motion."""
        x = self._radius * math.cos(self._omega * t)
        y = self._radius * math.sin(self._omega * t)
        yaw = self._omega * t + math.pi / 2.0
        vx = -self._radius * self._omega * math.sin(self._omega * t)
        vy = self._radius * self._omega * math.cos(self._omega * t)
        return x, y, yaw, vx, vy, self._omega

    def _true_accel(self, t: float):
        """Return centripetal (ax, ay) for circular motion."""
        ax = -(self._radius * self._omega ** 2
               * math.cos(self._omega * t))
        ay = -(self._radius * self._omega ** 2
               * math.sin(self._omega * t))
        return ax, ay

    def _publish_gps(self) -> None:
        """Publish noisy GPS position."""
        t = self._elapsed()
        x, y, _, _, _, _ = self._true_state(t)
        nx, ny, _ = add_gaussian_noise_3d(
            x, y, 0.0, self._gps_stddev, 0.0
        )
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.point.x = nx
        msg.point.y = ny
        msg.point.z = 0.0
        self._gps_pub.publish(msg)

    def _publish_imu(self) -> None:
        """Publish noisy IMU reading and update bias drift."""
        t = self._elapsed()
        x_t, y_t, yaw, _, _, yaw_rate = self._true_state(t)
        ax_true, ay_true = self._true_accel(t)

        self._accel_bias = drift_walk(self._accel_bias, 0.001, 0.05)
        self._gyro_bias = drift_walk(self._gyro_bias, 0.0005, 0.01)

        n_ax, n_ay, n_az, n_gz = generate_imu_noise(
            ax_true, ay_true, 0.0, yaw_rate,
            self._accel_stddev, self._gyro_stddev,
            accel_bias=self._accel_bias,
            gyro_bias=self._gyro_bias,
        )

        qx, qy, qz, qw = _quat_from_yaw(yaw)

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._base_frame_id
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        msg.angular_velocity.z = n_gz
        msg.linear_acceleration.x = n_ax
        msg.linear_acceleration.y = n_ay
        msg.linear_acceleration.z = n_az
        self._imu_pub.publish(msg)

    def _publish_odom(self) -> None:
        """Publish noisy wheel odometry and exact ground truth."""
        t = self._elapsed()
        x, y, yaw, vx, vy, yaw_rate = self._true_state(t)

        nx, ny, _ = add_gaussian_noise_3d(
            x, y, 0.0, self._odom_stddev, 0.0
        )
        n_vx = vx + random.gauss(0.0, self._odom_stddev)
        n_vy = vy + random.gauss(0.0, self._odom_stddev)

        qx, qy, qz, qw = _quat_from_yaw(yaw)
        stamp = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._frame_id
        odom.child_frame_id = self._base_frame_id
        odom.pose.pose.position.x = nx
        odom.pose.pose.position.y = ny
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = n_vx
        odom.twist.twist.linear.y = n_vy
        odom.twist.twist.angular.z = yaw_rate
        self._odom_pub.publish(odom)

        gt = Odometry()
        gt.header.stamp = stamp
        gt.header.frame_id = self._frame_id
        gt.child_frame_id = self._base_frame_id + '_truth'
        gt.pose.pose.position.x = x
        gt.pose.pose.position.y = y
        gt.pose.pose.orientation.x = qx
        gt.pose.pose.orientation.y = qy
        gt.pose.pose.orientation.z = qz
        gt.pose.pose.orientation.w = qw
        gt.twist.twist.linear.x = vx
        gt.twist.twist.linear.y = vy
        gt.twist.twist.angular.z = yaw_rate
        self._gt_pub.publish(gt)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self._frame_id
        transform.child_frame_id = self._base_frame_id + '_truth'
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(transform)


def main(args=None) -> None:
    """Entry point."""
    rclpy.init(args=args)
    node = NoisySensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

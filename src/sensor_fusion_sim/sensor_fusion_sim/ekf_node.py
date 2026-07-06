"""Extended Kalman Filter node fusing GPS, IMU, and wheel odometry."""

import math
import threading

from geometry_msgs.msg import PointStamped, Quaternion
from nav_msgs.msg import Odometry
import numpy as np
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_fusion_sim.ekf_math import (
    gps_measurement,
    imu_gyro_measurement,
    imu_yaw_measurement,
    initial_covariance,
    initial_state,
    make_gps_noise,
    make_imu_gyro_noise,
    make_imu_yaw_noise,
    make_odom_noise,
    make_process_noise,
    normalize_angle,
    odom_measurement,
    predict,
    update,
)
from sensor_msgs.msg import Imu
from std_msgs.msg import String


_STDDEV_PARAMS = (
    'process_pos_stddev', 'process_yaw_stddev',
    'process_vel_stddev', 'process_yaw_rate_stddev',
    'gps_pos_stddev', 'odom_pos_stddev', 'odom_vel_stddev',
    'imu_gyro_stddev', 'imu_yaw_stddev',
    'init_pos_stddev', 'init_yaw_stddev',
    'init_vel_stddev', 'init_yaw_rate_stddev',
)
_PROCESS_PARAMS = (
    'process_pos_stddev', 'process_yaw_stddev',
    'process_vel_stddev', 'process_yaw_rate_stddev',
)
_MEASUREMENT_PARAMS = (
    'gps_pos_stddev', 'odom_pos_stddev', 'odom_vel_stddev',
    'imu_gyro_stddev', 'imu_yaw_stddev',
)


class EkfNode(Node):
    """Fuse GPS, IMU, and wheel odometry with an Extended Kalman Filter."""

    def __init__(self) -> None:
        super().__init__('ekf_node')

        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('child_frame_id', 'base_link_ekf')
        self.declare_parameter('use_imu_orientation', True)
        self.declare_parameter('process_pos_stddev', 0.01)
        self.declare_parameter('process_yaw_stddev', 0.01)
        self.declare_parameter('process_vel_stddev', 0.1)
        self.declare_parameter('process_yaw_rate_stddev', 0.05)
        self.declare_parameter('gps_pos_stddev', 0.5)
        self.declare_parameter('odom_pos_stddev', 0.05)
        self.declare_parameter('odom_vel_stddev', 0.1)
        self.declare_parameter('imu_gyro_stddev', 0.02)
        self.declare_parameter('imu_yaw_stddev', 0.01)
        self.declare_parameter('init_pos_stddev', 1.0)
        self.declare_parameter('init_yaw_stddev', 0.5)
        self.declare_parameter('init_vel_stddev', 0.5)
        self.declare_parameter('init_yaw_rate_stddev', 0.1)

        publish_rate = float(
            self.get_parameter('publish_rate_hz').value
        )
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._child_frame_id = str(
            self.get_parameter('child_frame_id').value
        )
        self._use_imu_orientation = bool(
            self.get_parameter('use_imu_orientation').value
        )
        self._process_pos_stddev = float(
            self.get_parameter('process_pos_stddev').value
        )
        self._process_yaw_stddev = float(
            self.get_parameter('process_yaw_stddev').value
        )
        self._process_vel_stddev = float(
            self.get_parameter('process_vel_stddev').value
        )
        self._process_yaw_rate_stddev = float(
            self.get_parameter('process_yaw_rate_stddev').value
        )
        self._gps_pos_stddev = float(
            self.get_parameter('gps_pos_stddev').value
        )
        self._odom_pos_stddev = float(
            self.get_parameter('odom_pos_stddev').value
        )
        self._odom_vel_stddev = float(
            self.get_parameter('odom_vel_stddev').value
        )
        self._imu_gyro_stddev = float(
            self.get_parameter('imu_gyro_stddev').value
        )
        self._imu_yaw_stddev = float(
            self.get_parameter('imu_yaw_stddev').value
        )
        self._init_pos_stddev = float(
            self.get_parameter('init_pos_stddev').value
        )
        self._init_yaw_stddev = float(
            self.get_parameter('init_yaw_stddev').value
        )
        self._init_vel_stddev = float(
            self.get_parameter('init_vel_stddev').value
        )
        self._init_yaw_rate_stddev = float(
            self.get_parameter('init_yaw_rate_stddev').value
        )

        self._lock = threading.Lock()
        self._x = initial_state()
        self._P = initial_covariance(
            self._init_pos_stddev, self._init_yaw_stddev,
            self._init_vel_stddev, self._init_yaw_rate_stddev,
        )
        self._rebuild_process_noise()
        self._rebuild_measurement_noise()
        self._last_predict_time = None
        self._initialized = False
        self._gps_count = 0
        self._odom_count = 0
        self._imu_count = 0

        sensor_cb_group = ReentrantCallbackGroup()
        timer_cb_group = MutuallyExclusiveCallbackGroup()

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

        self.create_subscription(
            PointStamped, 'gps', self._on_gps, reliable_qos,
            callback_group=sensor_cb_group,
        )
        self.create_subscription(
            Imu, 'imu', self._on_imu, best_effort_qos,
            callback_group=sensor_cb_group,
        )
        self.create_subscription(
            Odometry, 'wheel_odom', self._on_odom, reliable_qos,
            callback_group=sensor_cb_group,
        )

        self._ekf_pub = self.create_publisher(
            Odometry, 'ekf_odom', reliable_qos,
        )
        self._diag_pub = self.create_publisher(
            String, 'ekf_diagnostics', reliable_qos,
        )

        period = 1.0 / max(publish_rate, 0.1)
        self.create_timer(
            period, self._publish_ekf,
            callback_group=timer_cb_group,
        )

        self.add_on_set_parameters_callback(
            self._on_param_change
        )

        self.get_logger().info(
            f'EkfNode started: frame_id={self._frame_id}, '
            f'child_frame_id={self._child_frame_id}'
        )

    def _rebuild_process_noise(self) -> None:
        self._Q = make_process_noise(
            self._process_pos_stddev, self._process_yaw_stddev,
            self._process_vel_stddev, self._process_yaw_rate_stddev,
        )

    def _rebuild_measurement_noise(self) -> None:
        self._R_gps = make_gps_noise(self._gps_pos_stddev)
        self._R_odom = make_odom_noise(
            self._odom_pos_stddev, self._odom_vel_stddev,
        )
        self._R_imu_gyro = make_imu_gyro_noise(self._imu_gyro_stddev)
        self._R_imu_yaw = make_imu_yaw_noise(self._imu_yaw_stddev)

    def _on_param_change(
        self, params: list,
    ) -> SetParametersResult:
        """Validate and apply dynamic parameter changes."""
        for param in params:
            if param.name in _STDDEV_PARAMS and float(param.value) <= 0.0:
                return SetParametersResult(
                    successful=False,
                    reason=f'{param.name} must be > 0',
                )

        with self._lock:
            rebuild_process = False
            rebuild_measurement = False
            for param in params:
                if param.name == 'frame_id':
                    self._frame_id = str(param.value)
                elif param.name == 'child_frame_id':
                    self._child_frame_id = str(param.value)
                elif param.name == 'use_imu_orientation':
                    self._use_imu_orientation = bool(param.value)
                elif param.name == 'process_pos_stddev':
                    self._process_pos_stddev = float(param.value)
                    rebuild_process = True
                elif param.name == 'process_yaw_stddev':
                    self._process_yaw_stddev = float(param.value)
                    rebuild_process = True
                elif param.name == 'process_vel_stddev':
                    self._process_vel_stddev = float(param.value)
                    rebuild_process = True
                elif param.name == 'process_yaw_rate_stddev':
                    self._process_yaw_rate_stddev = float(param.value)
                    rebuild_process = True
                elif param.name == 'gps_pos_stddev':
                    self._gps_pos_stddev = float(param.value)
                    rebuild_measurement = True
                elif param.name == 'odom_pos_stddev':
                    self._odom_pos_stddev = float(param.value)
                    rebuild_measurement = True
                elif param.name == 'odom_vel_stddev':
                    self._odom_vel_stddev = float(param.value)
                    rebuild_measurement = True
                elif param.name == 'imu_gyro_stddev':
                    self._imu_gyro_stddev = float(param.value)
                    rebuild_measurement = True
                elif param.name == 'imu_yaw_stddev':
                    self._imu_yaw_stddev = float(param.value)
                    rebuild_measurement = True
                elif param.name == 'init_pos_stddev':
                    self._init_pos_stddev = float(param.value)
                elif param.name == 'init_yaw_stddev':
                    self._init_yaw_stddev = float(param.value)
                elif param.name == 'init_vel_stddev':
                    self._init_vel_stddev = float(param.value)
                elif param.name == 'init_yaw_rate_stddev':
                    self._init_yaw_rate_stddev = float(param.value)

            if rebuild_process:
                self._rebuild_process_noise()
                self.get_logger().info('EKF process noise updated')
            if rebuild_measurement:
                self._rebuild_measurement_noise()
                self.get_logger().info('EKF measurement noise updated')

        return SetParametersResult(successful=True)

    def _predict_step(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if self._last_predict_time is not None:
            dt = (now_ns - self._last_predict_time) / 1e9
            if dt > 0:
                self._x, self._P = predict(
                    self._x, self._P, self._Q, dt,
                )
        self._last_predict_time = now_ns

    def _on_gps(self, msg: PointStamped) -> None:
        with self._lock:
            self._predict_step()
            self._initialized = True
            self._gps_count += 1
            z, h = gps_measurement(msg.point.x, msg.point.y)
            self._x, self._P = update(
                self._x, self._P, z, h, self._R_gps,
            )

    def _on_odom(self, msg: Odometry) -> None:
        with self._lock:
            self._predict_step()
            self._initialized = True
            self._odom_count += 1
            pos = msg.pose.pose.position
            z, h = odom_measurement(
                pos.x, pos.y, msg.twist.twist.linear.x,
            )
            self._x, self._P = update(
                self._x, self._P, z, h, self._R_odom,
            )

    def _on_imu(self, msg: Imu) -> None:
        with self._lock:
            self._predict_step()
            self._initialized = True
            self._imu_count += 1
            z, h = imu_gyro_measurement(msg.angular_velocity.z)
            self._x, self._P = update(
                self._x, self._P, z, h, self._R_imu_gyro,
            )
            if self._use_imu_orientation:
                yaw = _yaw_from_quaternion(msg.orientation)
                z, h = imu_yaw_measurement(yaw)
                self._x, self._P = update(
                    self._x, self._P, z, h, self._R_imu_yaw,
                    angle_indices=[0],
                )

    def _publish_ekf(self) -> None:
        with self._lock:
            x = self._x
            p = self._P
            gps_count = self._gps_count
            odom_count = self._odom_count
            imu_count = self._imu_count

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self._frame_id
        odom.child_frame_id = self._child_frame_id
        odom.pose.pose.position.x = float(x[0])
        odom.pose.pose.position.y = float(x[1])
        odom.pose.pose.orientation = _yaw_to_quaternion(
            normalize_angle(float(x[2]))
        )
        odom.twist.twist.linear.x = float(x[3])
        odom.twist.twist.angular.z = float(x[4])

        pose_cov = [0.0] * 36
        pose_cov[0] = float(p[0, 0])
        pose_cov[1] = float(p[0, 1])
        pose_cov[5] = float(p[0, 2])
        pose_cov[6] = float(p[1, 0])
        pose_cov[7] = float(p[1, 1])
        pose_cov[11] = float(p[1, 2])
        pose_cov[30] = float(p[2, 0])
        pose_cov[31] = float(p[2, 1])
        pose_cov[35] = float(p[2, 2])
        odom.pose.covariance = pose_cov

        twist_cov = [0.0] * 36
        twist_cov[0] = float(p[3, 3])
        twist_cov[5] = float(p[3, 4])
        twist_cov[30] = float(p[4, 3])
        twist_cov[35] = float(p[4, 4])
        odom.twist.covariance = twist_cov

        self._ekf_pub.publish(odom)

        diag = String()
        diag.data = (
            f'EKF | gps:{gps_count} odom:{odom_count} '
            f'imu:{imu_count} | '
            f'pos:({float(x[0]):.2f},{float(x[1]):.2f}) '
            f'yaw:{float(x[2]):.2f} v:{float(x[3]):.2f} | '
            f'trace(P):{float(np.trace(p)):.4f}'
        )
        self._diag_pub.publish(diag)


def _yaw_from_quaternion(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.z = math.sin(yaw / 2.0)
    return q


def main(args=None) -> None:
    """Entry point for the ekf_node executable."""
    rclpy.init(args=args)
    node = EkfNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

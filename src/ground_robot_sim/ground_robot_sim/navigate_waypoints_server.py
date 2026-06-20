"""Action server that drives the simulated ground robot through requested waypoints."""

import math
from math import atan2
from typing import List, Tuple

import rclpy
from geometry_msgs.msg import Twist
from ground_robot_sim.geometry import normalize_angle
from ground_robot_sim.pid import PIDController
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sample_interfaces.action import NavigateWaypoints


class NavigateWaypointsServer(Node):
    """Execute waypoint-navigation goals for the simulated differential-drive robot."""

    def __init__(self) -> None:
        super().__init__('navigate_waypoints_server')
        self.declare_parameter('max_linear_speed', 0.4)
        self.declare_parameter('max_angular_speed', 1.2)
        self.declare_parameter('kp_linear', 0.8)
        self.declare_parameter('ki_linear', 0.0)
        self.declare_parameter('kd_linear', 0.2)
        self.declare_parameter('kp_angular', 1.5)
        self.declare_parameter('ki_angular', 0.0)
        self.declare_parameter('kd_angular', 0.1)
        self.declare_parameter('heading_gate_rad', 0.5)
        self.declare_parameter('control_rate_hz', 20.0)

        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'odom', self._on_odom, 10)

        self._action_server = ActionServer(
            self,
            NavigateWaypoints,
            'navigate_waypoints',
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )
        self.get_logger().info('NavigateWaypointsServer ready')

    def _on_odom(self, msg: Odometry) -> None:
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self._yaw = 2.0 * atan2(q.z, q.w)

    def _goal_callback(self, goal_request: NavigateWaypoints.Goal) -> GoalResponse:
        if not goal_request.waypoints:
            self.get_logger().warn('Rejecting empty waypoints goal')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle) -> NavigateWaypoints.Result:
        waypoints: List[Tuple[float, float]] = [
            (pose_stamped.pose.position.x, pose_stamped.pose.position.y)
            for pose_stamped in goal_handle.request.waypoints
        ]
        loop = bool(goal_handle.request.loop)
        tolerance_m = max(0.01, float(goal_handle.request.tolerance_m or 0.15))

        max_linear = float(self.get_parameter('max_linear_speed').value)
        max_angular = float(self.get_parameter('max_angular_speed').value)
        kp_lin = float(self.get_parameter('kp_linear').value)
        ki_lin = float(self.get_parameter('ki_linear').value)
        kd_lin = float(self.get_parameter('kd_linear').value)
        kp_ang = float(self.get_parameter('kp_angular').value)
        ki_ang = float(self.get_parameter('ki_angular').value)
        kd_ang = float(self.get_parameter('kd_angular').value)
        heading_gate = float(self.get_parameter('heading_gate_rad').value)
        control_rate_hz = max(1.0, float(self.get_parameter('control_rate_hz').value))
        dt = 1.0 / control_rate_hz

        linear_pid = PIDController(
            kp_lin,
            ki_lin,
            kd_lin,
            output_min=0.0,
            output_max=max_linear,
        )
        angular_pid = PIDController(
            kp_ang,
            ki_ang,
            kd_ang,
            output_min=-max_angular,
            output_max=max_angular,
        )

        rate = self.create_rate(control_rate_hz)
        current_index = 0
        waypoints_completed = 0
        feedback = NavigateWaypoints.Feedback()

        self.get_logger().info(
            f'Executing NavigateWaypoints: {len(waypoints)} waypoints, loop={loop}'
        )

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self._stop_robot()
                goal_handle.canceled()
                result = NavigateWaypoints.Result()
                result.success = False
                result.waypoints_completed = waypoints_completed
                result.message = 'Goal cancelled'
                self.get_logger().info(
                    f'NavigateWaypoints cancelled after {waypoints_completed} waypoints'
                )
                return result

            if current_index >= len(waypoints):
                break

            target_x, target_y = waypoints[current_index]
            dx = target_x - self._x
            dy = target_y - self._y
            distance = math.sqrt(dx * dx + dy * dy)

            feedback.current_index = current_index
            feedback.total_waypoints = len(waypoints)
            feedback.distance_to_current = distance
            feedback.current_position.x = self._x
            feedback.current_position.y = self._y
            feedback.current_position.z = 0.0
            goal_handle.publish_feedback(feedback)

            if distance <= tolerance_m:
                waypoints_completed += 1
                self.get_logger().info(
                    f'Reached waypoint {current_index}: ({target_x:.3f}, {target_y:.3f})'
                )
                next_index = current_index + 1
                if next_index < len(waypoints):
                    current_index = next_index
                elif loop:
                    current_index = 0
                else:
                    current_index = len(waypoints)
                linear_pid.reset()
                angular_pid.reset()
                self._stop_robot()
            else:
                bearing = atan2(dy, dx)
                heading_error = normalize_angle(bearing - self._yaw)
                angular = angular_pid.compute(heading_error, dt)
                if abs(heading_error) <= heading_gate:
                    linear = linear_pid.compute(distance, dt)
                else:
                    linear = 0.0
                    linear_pid.reset()
                cmd = Twist()
                cmd.linear.x = linear
                cmd.angular.z = angular
                self._cmd_pub.publish(cmd)

            rate.sleep()

        self._stop_robot()
        goal_handle.succeed()
        result = NavigateWaypoints.Result()
        result.success = True
        result.waypoints_completed = waypoints_completed
        result.message = f'Completed {waypoints_completed} waypoint(s)'
        self.get_logger().info(result.message)
        return result

    def _stop_robot(self) -> None:
        self._cmd_pub.publish(Twist())


def main(args=None) -> None:
    """Initialise rclpy and spin the waypoint action server with a multithreaded executor."""
    rclpy.init(args=args)
    node = NavigateWaypointsServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

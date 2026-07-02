"""Mission state machine node coordinating takeoff, waypoint flight, RTL, and landing.

The pure transition rules (state + inputs -> next state) live in
``drone_sim.mission_logic`` and are unit tested there; this node only wires ROS
topics/services/parameters to that logic and drives a ``setpoint_pose`` target for
``sim_drone`` to chase.

During ``LAND`` this node continues to publish a descending ``setpoint_pose`` in
place, while the separate ``emergency_land`` node may independently publish
``cmd_vel`` descent commands in response to low battery. ``sim_drone`` prioritizes
``setpoint_pose`` over ``cmd_vel`` when both are fresh, but either strategy alone
converges on the same touchdown, so behavior stays consistent regardless of which
node is driving the descent.
"""

import math
from typing import Optional

import rclpy
from drone_sim.mission_logic import MissionConfig, MissionInputs, MissionState, next_state
from drone_sim.waypoint_utils import Point3, parse_waypoints
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class MissionStateMachine(Node):
    """Drive a drone through takeoff, waypoint mission, RTL, and landing states."""

    def __init__(self) -> None:
        super().__init__('mission_state_machine')

        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('takeoff_altitude_m', 1.5)
        self.declare_parameter(
            'waypoints',
            [2.0, 0.0, 1.5, 2.0, 2.0, 1.5, 0.0, 2.0, 1.5],
        )
        self.declare_parameter('tolerance_m', 0.25)
        self.declare_parameter('hold_time_sec', 1.0)
        self.declare_parameter('rtl_battery_pct', 30.0)
        self.declare_parameter('rtl_altitude_m', 1.5)
        self.declare_parameter('landed_altitude_m', 0.05)
        self.declare_parameter('auto_start', False)

        self.frame_id = self.get_parameter('frame_id').value
        self.takeoff_altitude_m = float(self.get_parameter('takeoff_altitude_m').value)
        self.waypoints = parse_waypoints(self.get_parameter('waypoints').value)
        self.tolerance_m = float(self.get_parameter('tolerance_m').value)
        self.hold_time_sec = float(self.get_parameter('hold_time_sec').value)
        self.rtl_battery_pct = float(self.get_parameter('rtl_battery_pct').value)
        self.rtl_altitude_m = float(self.get_parameter('rtl_altitude_m').value)
        self.landed_altitude_m = float(self.get_parameter('landed_altitude_m').value)
        self.auto_start = bool(self.get_parameter('auto_start').value)

        self.config = MissionConfig(
            takeoff_altitude_m=self.takeoff_altitude_m,
            rtl_battery_pct=self.rtl_battery_pct,
            arrival_tolerance_m=self.tolerance_m,
            landed_altitude_m=self.landed_altitude_m,
        )

        self.state = MissionState.IDLE
        self.current_position: Point3 = (0.0, 0.0, 0.0)
        self.battery_pct = 100.0
        self.critical_battery = False
        self.odom_received = False

        self.home_x = 0.0
        self.home_y = 0.0
        self.land_x = 0.0
        self.land_y = 0.0
        self.waypoint_index = 0
        self.mission_complete = False
        self.arrival_time = None

        self._start_requested = False
        self._abort_requested = False
        self._rtl_requested = False
        self._auto_start_done = False

        self.state_pub = self.create_publisher(String, 'mission_state', 10)
        self.setpoint_pub = self.create_publisher(PoseStamped, 'setpoint_pose', 10)

        self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self.create_subscription(BatteryState, 'battery', self._on_battery, 10)
        self.create_subscription(Bool, 'low_battery', self._on_low_battery, 10)

        self.create_service(Trigger, 'start_mission', self._on_start_mission)
        self.create_service(Trigger, 'abort_mission', self._on_abort_mission)
        self.create_service(Trigger, 'return_to_launch', self._on_return_to_launch)

        period = 1.0 / max(float(self.get_parameter('publish_rate_hz').value), 1.0)
        self.create_timer(period, self._tick)

        self.get_logger().info(
            f'MissionStateMachine ready with {len(self.waypoints)} waypoint(s), '
            f'takeoff_altitude={self.takeoff_altitude_m:.2f} m'
        )

    def _on_odom(self, msg: Odometry) -> None:
        """Track the current position from odometry."""
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        )
        if not self.odom_received:
            self.odom_received = True
            if self.auto_start and not self._auto_start_done:
                self._auto_start_done = True
                self._start_requested = True

    def _on_battery(self, msg: BatteryState) -> None:
        """Track the current battery percentage."""
        self.battery_pct = msg.percentage * 100.0

    def _on_low_battery(self, msg: Bool) -> None:
        """Track the latest critical battery flag."""
        self.critical_battery = msg.data

    def _on_start_mission(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Request a mission start from IDLE or LANDED."""
        if self.state in (MissionState.IDLE, MissionState.LANDED):
            self._start_requested = True
            response.success = True
            response.message = 'Mission start requested'
        else:
            response.success = False
            response.message = f'Cannot start mission from state {self.state.value}'
        return response

    def _on_abort_mission(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Request an abort-to-land from any airborne state."""
        if self.state in (MissionState.TAKEOFF, MissionState.MISSION, MissionState.RTL):
            self._abort_requested = True
            response.success = True
            response.message = 'Abort requested — landing'
        elif self.state == MissionState.LAND:
            response.success = True
            response.message = 'Already landing'
        else:
            response.success = False
            response.message = 'Not airborne'
        return response

    def _on_return_to_launch(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Request a return-to-launch from TAKEOFF or MISSION."""
        if self.state in (MissionState.TAKEOFF, MissionState.MISSION):
            self._rtl_requested = True
            response.success = True
            response.message = 'RTL requested'
        elif self.state == MissionState.RTL:
            response.success = True
            response.message = 'Already returning'
        else:
            response.success = False
            response.message = f'Cannot RTL from state {self.state.value}'
        return response

    def _current_target(self) -> Optional[Point3]:
        """Return the setpoint target for the current state, or None if idle."""
        if self.state == MissionState.TAKEOFF:
            return (self.home_x, self.home_y, self.takeoff_altitude_m)
        if self.state == MissionState.MISSION:
            return self.waypoints[self.waypoint_index]
        if self.state == MissionState.RTL:
            return (self.home_x, self.home_y, self.rtl_altitude_m)
        if self.state == MissionState.LAND:
            return (self.land_x, self.land_y, 0.0)
        return None

    def _advance_mission(self, target: Point3) -> None:
        """Advance the waypoint index / mission_complete flag when arrived and held."""
        now = self.get_clock().now()
        if math.dist(self.current_position, target) <= self.tolerance_m:
            if self.arrival_time is None:
                self.arrival_time = now
                self.get_logger().info(f'Reached waypoint {self.waypoint_index}: {target}')
            elif (now - self.arrival_time).nanoseconds * 1e-9 >= self.hold_time_sec:
                self.arrival_time = None
                if self.waypoint_index + 1 < len(self.waypoints):
                    self.waypoint_index += 1
                else:
                    self.mission_complete = True
        else:
            self.arrival_time = None

    def _enter_state(self, new_state: MissionState) -> None:
        """Run entry actions for a newly-entered state."""
        if new_state == MissionState.TAKEOFF:
            self.home_x, self.home_y, _ = self.current_position
            self.waypoint_index = 0
            self.mission_complete = False
            self.arrival_time = None
        elif new_state == MissionState.LAND:
            self.land_x, self.land_y, _ = self.current_position

    def _publish_state(self) -> None:
        """Publish the current mission state."""
        msg = String()
        msg.data = self.state.value
        self.state_pub.publish(msg)

    def _tick(self) -> None:
        """Evaluate the mission state machine and publish setpoints/state."""
        if self.state == MissionState.MISSION:
            self._advance_mission(self.waypoints[self.waypoint_index])

        target = self._current_target()
        distance_to_target_m = (
            math.dist(self.current_position, target) if target is not None else float('inf')
        )

        inputs = MissionInputs(
            altitude_m=self.current_position[2],
            battery_pct=self.battery_pct,
            distance_to_target_m=distance_to_target_m,
            mission_complete=self.mission_complete,
            start_requested=self._start_requested,
            abort_requested=self._abort_requested,
            rtl_requested=self._rtl_requested,
            critical_battery=self.critical_battery,
        )

        new_state, reason = next_state(self.state, inputs, self.config)
        if new_state != self.state:
            old_state = self.state
            self.state = new_state
            self.get_logger().info(f'{old_state.value} -> {new_state.value}: {reason}')
            self._enter_state(new_state)
            self._publish_state()
            target = self._current_target()

        if target is not None:
            now = self.get_clock().now()
            setpoint = PoseStamped()
            setpoint.header.stamp = now.to_msg()
            setpoint.header.frame_id = self.frame_id
            setpoint.pose.position.x = target[0]
            setpoint.pose.position.y = target[1]
            setpoint.pose.position.z = target[2]
            setpoint.pose.orientation.w = 1.0
            self.setpoint_pub.publish(setpoint)

        self._publish_state()

        self._start_requested = False
        self._abort_requested = False
        self._rtl_requested = False


def main(args=None) -> None:
    """Entry point for the mission_state_machine executable."""
    rclpy.init(args=args)
    node = MissionStateMachine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

"""
Behavior-tree mission node: the BT counterpart of ``mission_state_machine``.

The tree structure and all mission logic live in ``drone_sim.mission_bt``
(pure Python, unit tested); this node only wires ROS topics/services/
parameters to the blackboard and ticks the tree at a fixed rate.

It exposes the same interface as ``mission_state_machine`` — ``odom`` /
``battery`` / ``low_battery`` in, ``setpoint_pose`` / ``mission_state`` out,
``start_mission`` / ``abort_mission`` / ``return_to_launch`` services — so
the two nodes are drop-in replacements for each other in the mission demo.
In addition it publishes the per-tick path through the tree on ``bt_trace``
so learners can watch which branches run as the mission progresses.
"""

from drone_sim.bt_core import format_trace
from drone_sim.mission_bt import (
    build_mission_tree,
    MissionBlackboard,
    MissionBtConfig,
    tick_mission,
)
from drone_sim.waypoint_utils import parse_waypoints
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class MissionBehaviorTree(Node):
    """Drive the drone mission with a behavior tree instead of an FSM."""

    def __init__(self) -> None:
        super().__init__('mission_behavior_tree')

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
        self.declare_parameter('publish_trace', True)

        self.frame_id = self.get_parameter('frame_id').value
        self.auto_start = bool(self.get_parameter('auto_start').value)
        self.publish_trace = bool(self.get_parameter('publish_trace').value)

        config = MissionBtConfig(
            takeoff_altitude_m=float(self.get_parameter('takeoff_altitude_m').value),
            rtl_battery_pct=float(self.get_parameter('rtl_battery_pct').value),
            arrival_tolerance_m=float(self.get_parameter('tolerance_m').value),
            landed_altitude_m=float(self.get_parameter('landed_altitude_m').value),
            rtl_altitude_m=float(self.get_parameter('rtl_altitude_m').value),
            hold_time_sec=float(self.get_parameter('hold_time_sec').value),
            waypoints=tuple(parse_waypoints(self.get_parameter('waypoints').value)),
        )
        self.blackboard = MissionBlackboard(config=config)
        self.tree = build_mission_tree()

        self.odom_received = False
        self._auto_start_done = False
        self._start_requested = False
        self._abort_requested = False
        self._rtl_requested = False
        self._last_phase = self.blackboard.phase
        self._last_trace = ''

        self.state_pub = self.create_publisher(String, 'mission_state', 10)
        self.setpoint_pub = self.create_publisher(PoseStamped, 'setpoint_pose', 10)
        self.trace_pub = self.create_publisher(String, 'bt_trace', 10)

        self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self.create_subscription(BatteryState, 'battery', self._on_battery, 10)
        self.create_subscription(Bool, 'low_battery', self._on_low_battery, 10)

        self.create_service(Trigger, 'start_mission', self._on_start_mission)
        self.create_service(Trigger, 'abort_mission', self._on_abort_mission)
        self.create_service(Trigger, 'return_to_launch', self._on_return_to_launch)

        period = 1.0 / max(float(self.get_parameter('publish_rate_hz').value), 1.0)
        self.create_timer(period, self._tick)

        self.get_logger().info(
            f'MissionBehaviorTree ready with {len(config.waypoints)} waypoint(s), '
            f'takeoff_altitude={config.takeoff_altitude_m:.2f} m'
        )

    def _on_odom(self, msg: Odometry) -> None:
        """Track the current position from odometry."""
        self.blackboard.position = (
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
        self.blackboard.battery_pct = msg.percentage * 100.0

    def _on_low_battery(self, msg: Bool) -> None:
        """Track the latest critical battery flag."""
        self.blackboard.critical_battery = msg.data

    def _on_start_mission(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Request a mission start while no mission is active."""
        if not self.blackboard.active:
            self._start_requested = True
            response.success = True
            response.message = 'Mission start requested'
        else:
            response.success = False
            response.message = f'Cannot start mission from phase {self.blackboard.phase}'
        return response

    def _on_abort_mission(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Request an abort-to-land from any airborne phase."""
        if self.blackboard.phase in ('TAKEOFF', 'MISSION', 'RTL'):
            self._abort_requested = True
            response.success = True
            response.message = 'Abort requested — landing'
        elif self.blackboard.phase == 'LAND':
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
        if self.blackboard.phase in ('TAKEOFF', 'MISSION'):
            self._rtl_requested = True
            response.success = True
            response.message = 'RTL requested'
        elif self.blackboard.phase == 'RTL':
            response.success = True
            response.message = 'Already returning'
        else:
            response.success = False
            response.message = f'Cannot RTL from phase {self.blackboard.phase}'
        return response

    def _tick(self) -> None:
        """Feed inputs to the blackboard, tick the tree, and publish outputs."""
        bb = self.blackboard
        bb.start_requested = self._start_requested
        bb.abort_requested = self._abort_requested
        bb.rtl_requested = self._rtl_requested
        bb.now_sec = self.get_clock().now().nanoseconds * 1e-9

        tick_mission(self.tree, bb)

        if bb.phase != self._last_phase:
            self.get_logger().info(f'{self._last_phase} -> {bb.phase}')
            self._last_phase = bb.phase

        if bb.setpoint is not None:
            setpoint = PoseStamped()
            setpoint.header.stamp = self.get_clock().now().to_msg()
            setpoint.header.frame_id = self.frame_id
            setpoint.pose.position.x = bb.setpoint[0]
            setpoint.pose.position.y = bb.setpoint[1]
            setpoint.pose.position.z = bb.setpoint[2]
            setpoint.pose.orientation.w = 1.0
            self.setpoint_pub.publish(setpoint)

        state_msg = String()
        state_msg.data = bb.phase
        self.state_pub.publish(state_msg)

        if self.publish_trace:
            trace_str = format_trace(bb.trace)
            if trace_str != self._last_trace:
                self._last_trace = trace_str
                trace_msg = String()
                trace_msg.data = trace_str
                self.trace_pub.publish(trace_msg)

        self._start_requested = False
        self._abort_requested = False
        self._rtl_requested = False


def main(args=None) -> None:
    """Entry point for the mission_behavior_tree executable."""
    rclpy.init(args=args)
    node = MissionBehaviorTree()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

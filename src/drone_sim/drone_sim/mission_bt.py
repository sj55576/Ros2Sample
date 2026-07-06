"""
Pure behavior-tree mission logic for the drone simulation (no ROS imports).

This mirrors the mission implemented as a finite state machine in
``drone_sim.mission_logic`` — takeoff, waypoint visiting, return to launch,
and landing, with battery / abort interrupts — but expresses it as a
behavior tree built on ``drone_sim.bt_core``.

Key design notes for comparing with the FSM version:

* Priority as tree order: the emergency-landing branch is simply the first
  child of the flight Selector, so it preempts every other branch on any
  tick. The FSM needs an explicit LAND transition out of every airborne
  state to express the same thing.
* Latches instead of states: a reactive tree re-evaluates everything each
  tick, so one-shot events (abort request, "landing has started") are
  remembered as blackboard flags (``emergency_land``, ``rtl_latched``,
  ``landing``, ``takeoff_done``). These flags are the BT equivalent of the
  memory that an FSM keeps implicitly in its current state.
* The blackboard is the single mutable context: inputs are written by the
  caller before each tick, and outputs (``setpoint``, ``phase``) are read
  back after the tick.

``phase`` intentionally uses the same labels as ``MissionState`` so the two
implementations can be observed side by side on the ``mission_state`` topic.
"""

from dataclasses import dataclass, field
import math
from typing import List, Optional, Tuple

from drone_sim.bt_core import (
    Action,
    BehaviorNode,
    Condition,
    Selector,
    Sequence,
    Status,
)

Point3 = Tuple[float, float, float]


@dataclass(frozen=True)
class MissionBtConfig:
    """Static thresholds and the waypoint list used by the mission tree."""

    takeoff_altitude_m: float = 1.5
    rtl_battery_pct: float = 30.0
    arrival_tolerance_m: float = 0.25
    landed_altitude_m: float = 0.05
    rtl_altitude_m: float = 1.5
    hold_time_sec: float = 1.0
    waypoints: Tuple[Point3, ...] = ()


@dataclass
class MissionBlackboard:
    """
    Shared mutable context read and written by every tree leaf.

    Inputs are refreshed by the caller before each tick; state fields are
    owned by the tree; outputs are read back by the caller after each tick.
    """

    config: MissionBtConfig = field(default_factory=MissionBtConfig)

    # --- inputs (written by the caller before each tick) ---
    position: Point3 = (0.0, 0.0, 0.0)
    battery_pct: float = 100.0
    critical_battery: bool = False
    start_requested: bool = False
    abort_requested: bool = False
    rtl_requested: bool = False
    now_sec: float = 0.0

    # --- state (owned by the tree, reset on mission start) ---
    active: bool = False
    emergency_land: bool = False
    rtl_latched: bool = False
    landing: bool = False
    takeoff_done: bool = False
    mission_complete: bool = False
    waypoint_index: int = 0
    arrival_time_sec: Optional[float] = None
    home_x: float = 0.0
    home_y: float = 0.0
    land_target: Optional[Tuple[float, float]] = None

    # --- outputs (read by the caller after each tick) ---
    setpoint: Optional[Point3] = None
    phase: str = 'IDLE'
    trace: List[Tuple[str, Status]] = field(default_factory=list)


def _activate(bb: MissionBlackboard) -> None:
    """Reset all mission state and start a new flight from the current position."""
    bb.active = True
    bb.emergency_land = False
    bb.rtl_latched = False
    bb.landing = False
    bb.takeoff_done = False
    bb.mission_complete = False
    bb.waypoint_index = 0
    bb.arrival_time_sec = None
    bb.home_x = bb.position[0]
    bb.home_y = bb.position[1]
    bb.land_target = None
    bb.phase = 'TAKEOFF'


def _deactivate(bb: MissionBlackboard) -> None:
    """Finish the flight after touchdown."""
    bb.active = False
    bb.setpoint = None
    bb.phase = 'LANDED'


def _is_active(bb: MissionBlackboard) -> bool:
    """Gate the whole flight subtree on a mission being in progress."""
    return bb.active


def _emergency(bb: MissionBlackboard) -> bool:
    """Latch an emergency landing on critical battery or an abort request."""
    if bb.critical_battery or bb.abort_requested:
        bb.emergency_land = True
    return bb.emergency_land


def _rtl_required(bb: MissionBlackboard) -> bool:
    """Latch return-to-launch on low battery or an RTL request; also fire on completion."""
    if not bb.emergency_land:
        if bb.rtl_requested or bb.battery_pct <= bb.config.rtl_battery_pct:
            bb.rtl_latched = True
    return bb.rtl_latched or bb.mission_complete


def _home_reached_or_landing(bb: MissionBlackboard) -> bool:
    """Latch the final landing once the drone has reached the point above home."""
    if not bb.landing:
        home = (bb.home_x, bb.home_y, bb.config.rtl_altitude_m)
        if math.dist(bb.position, home) <= bb.config.arrival_tolerance_m:
            bb.landing = True
    return bb.landing


def _land(bb: MissionBlackboard) -> Status:
    """Descend in place (capturing the touchdown point once) until landed."""
    if bb.land_target is None:
        bb.land_target = (bb.position[0], bb.position[1])
    bb.phase = 'LAND'
    bb.setpoint = (bb.land_target[0], bb.land_target[1], 0.0)
    if bb.position[2] <= bb.config.landed_altitude_m:
        _deactivate(bb)
        return Status.SUCCESS
    return Status.RUNNING


def _fly_home(bb: MissionBlackboard) -> Status:
    """Head back to the launch point at RTL altitude."""
    bb.phase = 'RTL'
    bb.setpoint = (bb.home_x, bb.home_y, bb.config.rtl_altitude_m)
    return Status.RUNNING


def _need_takeoff(bb: MissionBlackboard) -> bool:
    """Latch takeoff completion so a later altitude dip does not restart the climb."""
    threshold = bb.config.takeoff_altitude_m - bb.config.arrival_tolerance_m
    if bb.position[2] >= threshold:
        bb.takeoff_done = True
    return not bb.takeoff_done


def _climb(bb: MissionBlackboard) -> Status:
    """Climb straight up over the launch point to takeoff altitude."""
    bb.phase = 'TAKEOFF'
    bb.setpoint = (bb.home_x, bb.home_y, bb.config.takeoff_altitude_m)
    return Status.RUNNING


def _visit_waypoints(bb: MissionBlackboard) -> Status:
    """Chase the current waypoint, holding at each one before advancing."""
    if not bb.config.waypoints or bb.mission_complete:
        bb.mission_complete = True
        return Status.SUCCESS

    target = bb.config.waypoints[bb.waypoint_index]
    if math.dist(bb.position, target) <= bb.config.arrival_tolerance_m:
        if bb.arrival_time_sec is None:
            bb.arrival_time_sec = bb.now_sec
        elif bb.now_sec - bb.arrival_time_sec >= bb.config.hold_time_sec:
            bb.arrival_time_sec = None
            if bb.waypoint_index + 1 < len(bb.config.waypoints):
                bb.waypoint_index += 1
                target = bb.config.waypoints[bb.waypoint_index]
            else:
                bb.mission_complete = True
                return Status.SUCCESS
    else:
        bb.arrival_time_sec = None

    bb.phase = 'MISSION'
    bb.setpoint = target
    return Status.RUNNING


def _await_start(bb: MissionBlackboard) -> Status:
    """Idle on the ground; arm a fresh mission when a start is requested."""
    bb.setpoint = None
    if bb.start_requested:
        _activate(bb)
    return Status.SUCCESS


def build_mission_tree() -> BehaviorNode:
    """
    Assemble the mission behavior tree.

    Tree shape (higher child = higher priority)::

        Selector "mission_root"
        ├── Sequence "flight"
        │   ├── Condition "is_active?"
        │   └── Selector "flight_modes"
        │       ├── Sequence "emergency"
        │       │   ├── Condition "emergency?"       (abort / critical battery)
        │       │   └── Action    "land_in_place"
        │       ├── Sequence "return_home"
        │       │   ├── Condition "rtl_required?"    (low battery / RTL / complete)
        │       │   └── Selector  "rtl_then_land"
        │       │       ├── Sequence "land_at_home"
        │       │       │   ├── Condition "home_reached?"
        │       │       │   └── Action    "final_land"
        │       │       └── Action "fly_home"
        │       ├── Sequence "takeoff"
        │       │   ├── Condition "need_takeoff?"
        │       │   └── Action    "climb"
        │       └── Action "visit_waypoints"
        └── Action "await_start"
    """
    return Selector('mission_root', [
        Sequence('flight', [
            Condition('is_active?', _is_active),
            Selector('flight_modes', [
                Sequence('emergency', [
                    Condition('emergency?', _emergency),
                    Action('land_in_place', _land),
                ]),
                Sequence('return_home', [
                    Condition('rtl_required?', _rtl_required),
                    Selector('rtl_then_land', [
                        Sequence('land_at_home', [
                            Condition('home_reached?', _home_reached_or_landing),
                            Action('final_land', _land),
                        ]),
                        Action('fly_home', _fly_home),
                    ]),
                ]),
                Sequence('takeoff', [
                    Condition('need_takeoff?', _need_takeoff),
                    Action('climb', _climb),
                ]),
                Action('visit_waypoints', _visit_waypoints),
            ]),
        ]),
        Action('await_start', _await_start),
    ])


def tick_mission(tree: BehaviorNode, bb: MissionBlackboard) -> Status:
    """Run one tick: reset the trace, tick the tree, and return the root status."""
    bb.trace = []
    return tree.tick(bb)

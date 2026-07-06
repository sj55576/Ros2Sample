"""
Scenario tests for the behavior-tree mission logic.

These mirror the transition tests of ``test_mission_logic`` (the FSM
version) so the two implementations can be checked against the same
mission semantics.
"""

from drone_sim.bt_core import Status
from drone_sim.mission_bt import (
    build_mission_tree,
    MissionBlackboard,
    MissionBtConfig,
    tick_mission,
)

WAYPOINTS = ((2.0, 0.0, 1.5), (2.0, 2.0, 1.5))


def make_bb(**config_kwargs):
    """Return a fresh blackboard with the default test waypoints."""
    config_kwargs.setdefault('waypoints', WAYPOINTS)
    return MissionBlackboard(config=MissionBtConfig(**config_kwargs))


def tick(tree, bb, **inputs):
    """Set one-shot inputs, tick once, then clear the one-shot flags."""
    for key, value in inputs.items():
        setattr(bb, key, value)
    status = tick_mission(tree, bb)
    bb.start_requested = False
    bb.abort_requested = False
    bb.rtl_requested = False
    return status


def start_mission(tree, bb, position=(0.0, 0.0, 0.0)):
    """Drive the tree through a mission start from the given launch point."""
    bb.position = position
    tick(tree, bb, start_requested=True)
    tick(tree, bb)


class TestIdle:

    def test_stays_idle_without_start(self):
        tree, bb = build_mission_tree(), make_bb()
        assert tick(tree, bb) == Status.SUCCESS
        assert bb.phase == 'IDLE'
        assert bb.setpoint is None

    def test_ignores_abort_rtl_and_battery_on_ground(self):
        tree, bb = build_mission_tree(), make_bb()
        tick(tree, bb, abort_requested=True, rtl_requested=True,
             critical_battery=True, battery_pct=1.0)
        assert bb.phase == 'IDLE'
        assert not bb.active
        bb.critical_battery = False
        bb.battery_pct = 100.0

    def test_start_enters_takeoff_and_captures_home(self):
        tree, bb = build_mission_tree(), make_bb()
        bb.position = (0.5, -0.5, 0.0)
        tick(tree, bb, start_requested=True)
        assert bb.active
        assert bb.phase == 'TAKEOFF'
        assert (bb.home_x, bb.home_y) == (0.5, -0.5)
        tick(tree, bb)
        assert bb.setpoint == (0.5, -0.5, 1.5)


class TestTakeoffAndMission:

    def test_climb_then_visit_waypoints(self):
        tree, bb = build_mission_tree(), make_bb()
        start_mission(tree, bb)
        assert bb.phase == 'TAKEOFF'
        bb.position = (0.0, 0.0, 1.5)
        tick(tree, bb)
        assert bb.phase == 'MISSION'
        assert bb.setpoint == WAYPOINTS[0]

    def test_takeoff_latch_prevents_reclimb_on_dip(self):
        tree, bb = build_mission_tree(), make_bb()
        start_mission(tree, bb)
        bb.position = (0.0, 0.0, 1.5)
        tick(tree, bb)
        bb.position = (1.0, 0.0, 1.0)  # altitude dips below the takeoff threshold
        tick(tree, bb)
        assert bb.phase == 'MISSION'
        assert bb.setpoint == WAYPOINTS[0]

    def test_waypoint_advance_requires_hold_time(self):
        tree, bb = build_mission_tree(), make_bb(hold_time_sec=1.0)
        start_mission(tree, bb)
        bb.position = (2.0, 0.0, 1.5)  # on waypoint 0 (also above takeoff altitude)
        tick(tree, bb, now_sec=10.0)
        assert bb.waypoint_index == 0
        tick(tree, bb, now_sec=10.5)
        assert bb.waypoint_index == 0
        tick(tree, bb, now_sec=11.1)
        assert bb.waypoint_index == 1
        assert bb.setpoint == WAYPOINTS[1]

    def test_leaving_tolerance_resets_hold_timer(self):
        tree, bb = build_mission_tree(), make_bb(hold_time_sec=1.0)
        start_mission(tree, bb)
        bb.position = (2.0, 0.0, 1.5)
        tick(tree, bb, now_sec=10.0)
        bb.position = (1.0, 0.0, 1.5)  # drift away before the hold elapses
        tick(tree, bb, now_sec=10.5)
        bb.position = (2.0, 0.0, 1.5)
        tick(tree, bb, now_sec=11.1)
        assert bb.waypoint_index == 0  # hold timer restarted

    def test_mission_complete_triggers_rtl(self):
        tree, bb = build_mission_tree(), make_bb(hold_time_sec=0.0)
        start_mission(tree, bb)
        bb.position = (2.0, 0.0, 1.5)
        tick(tree, bb, now_sec=1.0)
        tick(tree, bb, now_sec=2.0)
        assert bb.waypoint_index == 1
        bb.position = (2.0, 2.0, 1.5)
        tick(tree, bb, now_sec=3.0)
        tick(tree, bb, now_sec=4.0)
        assert bb.mission_complete
        tick(tree, bb, now_sec=5.0)
        assert bb.phase == 'RTL'
        assert bb.setpoint == (0.0, 0.0, 1.5)


class TestRtl:

    def test_low_battery_latches_rtl(self):
        tree, bb = build_mission_tree(), make_bb(rtl_battery_pct=30.0)
        start_mission(tree, bb)
        bb.position = (2.0, 0.0, 1.5)  # away from home so RTL flies instead of landing
        tick(tree, bb, battery_pct=25.0)
        assert bb.phase == 'RTL'
        bb.battery_pct = 35.0  # a recovering reading must not resume the mission
        tick(tree, bb)
        assert bb.phase == 'RTL'

    def test_rtl_request_latches(self):
        tree, bb = build_mission_tree(), make_bb()
        start_mission(tree, bb)
        bb.position = (1.0, 0.0, 1.5)
        tick(tree, bb, rtl_requested=True)
        assert bb.phase == 'RTL'
        tick(tree, bb)
        assert bb.phase == 'RTL'

    def test_home_reached_starts_landing_and_stays_landing(self):
        tree, bb = build_mission_tree(), make_bb()
        start_mission(tree, bb)
        bb.position = (1.0, 0.0, 1.5)
        tick(tree, bb, rtl_requested=True)
        bb.position = (0.0, 0.0, 1.5)  # back above home at RTL altitude
        tick(tree, bb)
        assert bb.phase == 'LAND'
        assert bb.setpoint == (0.0, 0.0, 0.0)
        bb.position = (0.0, 0.0, 0.8)  # descending: 3D distance to home now exceeds
        tick(tree, bb)                 # tolerance, but the landing latch must hold
        assert bb.phase == 'LAND'

    def test_touchdown_finishes_mission(self):
        tree, bb = build_mission_tree(), make_bb()
        start_mission(tree, bb)
        bb.position = (0.0, 0.0, 1.5)
        tick(tree, bb, rtl_requested=True)
        tick(tree, bb)
        bb.position = (0.0, 0.0, 0.02)
        tick(tree, bb)
        assert bb.phase == 'LANDED'
        assert not bb.active
        assert bb.setpoint is None


class TestEmergency:

    def test_abort_lands_in_place(self):
        tree, bb = build_mission_tree(), make_bb()
        start_mission(tree, bb)
        bb.position = (1.5, 0.7, 1.5)
        tick(tree, bb, abort_requested=True)
        assert bb.phase == 'LAND'
        assert bb.setpoint == (1.5, 0.7, 0.0)

    def test_critical_battery_preempts_rtl(self):
        tree, bb = build_mission_tree(), make_bb()
        start_mission(tree, bb)
        bb.position = (2.0, 0.0, 1.5)
        tick(tree, bb, rtl_requested=True)
        assert bb.phase == 'RTL'
        tick(tree, bb, critical_battery=True)
        assert bb.phase == 'LAND'
        assert bb.setpoint == (2.0, 0.0, 0.0)  # in place, not at home
        bb.critical_battery = False

    def test_emergency_touchdown_allows_restart(self):
        tree, bb = build_mission_tree(), make_bb()
        start_mission(tree, bb)
        bb.position = (1.0, 1.0, 1.5)
        tick(tree, bb, abort_requested=True)
        bb.position = (1.0, 1.0, 0.0)
        tick(tree, bb)
        assert bb.phase == 'LANDED'
        tick(tree, bb, start_requested=True)
        assert bb.phase == 'TAKEOFF'
        assert (bb.home_x, bb.home_y) == (1.0, 1.0)
        assert not bb.emergency_land


class TestTrace:

    def test_trace_shows_active_branch(self):
        tree, bb = build_mission_tree(), make_bb()
        tick(tree, bb)
        names = [name for name, _ in bb.trace]
        assert 'await_start' in names
        start_mission(tree, bb)
        names = [name for name, _ in bb.trace]
        assert 'climb' in names
        assert 'await_start' not in names

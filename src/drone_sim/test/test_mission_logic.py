"""Tests for the mission state-machine transition logic."""

from drone_sim.mission_logic import MissionConfig, MissionInputs, MissionState, next_state


class TestIdle:
    def test_stays_idle_with_default_inputs(self):
        state, reason = next_state(MissionState.IDLE, MissionInputs(), MissionConfig())
        assert state == MissionState.IDLE
        assert reason is None

    def test_start_requested_goes_to_takeoff(self):
        inputs = MissionInputs(start_requested=True)
        state, reason = next_state(MissionState.IDLE, inputs, MissionConfig())
        assert state == MissionState.TAKEOFF
        assert reason == 'start requested'

    def test_ignores_abort_requested(self):
        inputs = MissionInputs(abort_requested=True)
        state, reason = next_state(MissionState.IDLE, inputs, MissionConfig())
        assert state == MissionState.IDLE
        assert reason is None

    def test_ignores_rtl_requested(self):
        inputs = MissionInputs(rtl_requested=True)
        state, reason = next_state(MissionState.IDLE, inputs, MissionConfig())
        assert state == MissionState.IDLE
        assert reason is None

    def test_ignores_critical_battery(self):
        inputs = MissionInputs(critical_battery=True)
        state, reason = next_state(MissionState.IDLE, inputs, MissionConfig())
        assert state == MissionState.IDLE
        assert reason is None

    def test_ignores_low_battery_pct(self):
        inputs = MissionInputs(battery_pct=1.0)
        state, reason = next_state(MissionState.IDLE, inputs, MissionConfig())
        assert state == MissionState.IDLE
        assert reason is None


class TestLanded:
    def test_start_requested_goes_to_takeoff(self):
        inputs = MissionInputs(start_requested=True)
        state, reason = next_state(MissionState.LANDED, inputs, MissionConfig())
        assert state == MissionState.TAKEOFF
        assert reason == 'start requested'

    def test_stays_landed_with_default_inputs(self):
        state, reason = next_state(MissionState.LANDED, MissionInputs(), MissionConfig())
        assert state == MissionState.LANDED
        assert reason is None

    def test_ignores_abort_requested(self):
        inputs = MissionInputs(abort_requested=True)
        state, reason = next_state(MissionState.LANDED, inputs, MissionConfig())
        assert state == MissionState.LANDED
        assert reason is None

    def test_ignores_critical_battery(self):
        inputs = MissionInputs(critical_battery=True)
        state, reason = next_state(MissionState.LANDED, inputs, MissionConfig())
        assert state == MissionState.LANDED
        assert reason is None


class TestTakeoff:
    def test_reaches_mission_at_exact_threshold(self):
        config = MissionConfig()
        threshold = config.takeoff_altitude_m - config.arrival_tolerance_m
        inputs = MissionInputs(altitude_m=threshold)
        state, reason = next_state(MissionState.TAKEOFF, inputs, config)
        assert state == MissionState.MISSION
        assert reason == 'takeoff altitude reached'

    def test_reaches_mission_above_threshold(self):
        config = MissionConfig()
        threshold = config.takeoff_altitude_m - config.arrival_tolerance_m
        inputs = MissionInputs(altitude_m=threshold + 1.0)
        state, reason = next_state(MissionState.TAKEOFF, inputs, config)
        assert state == MissionState.MISSION
        assert reason == 'takeoff altitude reached'

    def test_stays_takeoff_below_threshold(self):
        config = MissionConfig()
        threshold = config.takeoff_altitude_m - config.arrival_tolerance_m
        inputs = MissionInputs(altitude_m=threshold - 0.01)
        state, reason = next_state(MissionState.TAKEOFF, inputs, config)
        assert state == MissionState.TAKEOFF
        assert reason is None

    def test_battery_pct_at_boundary_triggers_rtl(self):
        config = MissionConfig()
        inputs = MissionInputs(battery_pct=config.rtl_battery_pct)
        state, reason = next_state(MissionState.TAKEOFF, inputs, config)
        assert state == MissionState.RTL
        assert reason == 'low battery'

    def test_rtl_requested_goes_to_rtl(self):
        inputs = MissionInputs(rtl_requested=True)
        state, reason = next_state(MissionState.TAKEOFF, inputs, MissionConfig())
        assert state == MissionState.RTL
        assert reason == 'RTL requested'

    def test_abort_requested_goes_to_land(self):
        inputs = MissionInputs(abort_requested=True)
        state, reason = next_state(MissionState.TAKEOFF, inputs, MissionConfig())
        assert state == MissionState.LAND
        assert reason == 'abort requested'

    def test_critical_battery_goes_to_land(self):
        inputs = MissionInputs(critical_battery=True)
        state, reason = next_state(MissionState.TAKEOFF, inputs, MissionConfig())
        assert state == MissionState.LAND
        assert reason == 'critical battery'


class TestMission:
    def test_mission_complete_goes_to_rtl(self):
        inputs = MissionInputs(mission_complete=True)
        state, reason = next_state(MissionState.MISSION, inputs, MissionConfig())
        assert state == MissionState.RTL
        assert reason == 'mission complete'

    def test_stays_mission_with_default_inputs(self):
        state, reason = next_state(MissionState.MISSION, MissionInputs(), MissionConfig())
        assert state == MissionState.MISSION
        assert reason is None

    def test_battery_pct_at_boundary_triggers_rtl(self):
        config = MissionConfig()
        inputs = MissionInputs(battery_pct=config.rtl_battery_pct)
        state, reason = next_state(MissionState.MISSION, inputs, config)
        assert state == MissionState.RTL
        assert reason == 'low battery'

    def test_rtl_requested_goes_to_rtl(self):
        inputs = MissionInputs(rtl_requested=True)
        state, reason = next_state(MissionState.MISSION, inputs, MissionConfig())
        assert state == MissionState.RTL
        assert reason == 'RTL requested'

    def test_abort_requested_goes_to_land(self):
        inputs = MissionInputs(abort_requested=True)
        state, reason = next_state(MissionState.MISSION, inputs, MissionConfig())
        assert state == MissionState.LAND
        assert reason == 'abort requested'

    def test_critical_battery_goes_to_land(self):
        inputs = MissionInputs(critical_battery=True)
        state, reason = next_state(MissionState.MISSION, inputs, MissionConfig())
        assert state == MissionState.LAND
        assert reason == 'critical battery'


class TestRtl:
    def test_reaches_land_at_exact_boundary(self):
        config = MissionConfig()
        inputs = MissionInputs(distance_to_target_m=config.arrival_tolerance_m)
        state, reason = next_state(MissionState.RTL, inputs, config)
        assert state == MissionState.LAND
        assert reason == 'home position reached'

    def test_reaches_land_below_boundary(self):
        config = MissionConfig()
        inputs = MissionInputs(distance_to_target_m=config.arrival_tolerance_m - 0.1)
        state, reason = next_state(MissionState.RTL, inputs, config)
        assert state == MissionState.LAND
        assert reason == 'home position reached'

    def test_stays_rtl_above_boundary(self):
        config = MissionConfig()
        inputs = MissionInputs(distance_to_target_m=config.arrival_tolerance_m + 0.1)
        state, reason = next_state(MissionState.RTL, inputs, config)
        assert state == MissionState.RTL
        assert reason is None

    def test_ignores_low_battery_pct(self):
        config = MissionConfig()
        inputs = MissionInputs(battery_pct=1.0)
        state, reason = next_state(MissionState.RTL, inputs, config)
        assert state == MissionState.RTL
        assert reason is None

    def test_ignores_rtl_requested(self):
        inputs = MissionInputs(rtl_requested=True)
        state, reason = next_state(MissionState.RTL, inputs, MissionConfig())
        assert state == MissionState.RTL
        assert reason is None

    def test_abort_requested_goes_to_land(self):
        inputs = MissionInputs(abort_requested=True)
        state, reason = next_state(MissionState.RTL, inputs, MissionConfig())
        assert state == MissionState.LAND
        assert reason == 'abort requested'

    def test_critical_battery_goes_to_land(self):
        inputs = MissionInputs(critical_battery=True)
        state, reason = next_state(MissionState.RTL, inputs, MissionConfig())
        assert state == MissionState.LAND
        assert reason == 'critical battery'


class TestLand:
    def test_reaches_landed_at_exact_boundary(self):
        config = MissionConfig()
        inputs = MissionInputs(altitude_m=config.landed_altitude_m)
        state, reason = next_state(MissionState.LAND, inputs, config)
        assert state == MissionState.LANDED
        assert reason == 'touchdown'

    def test_reaches_landed_below_boundary(self):
        config = MissionConfig()
        inputs = MissionInputs(altitude_m=config.landed_altitude_m - 0.01)
        state, reason = next_state(MissionState.LAND, inputs, config)
        assert state == MissionState.LANDED
        assert reason == 'touchdown'

    def test_stays_land_above_boundary(self):
        config = MissionConfig()
        inputs = MissionInputs(altitude_m=config.landed_altitude_m + 0.1)
        state, reason = next_state(MissionState.LAND, inputs, config)
        assert state == MissionState.LAND
        assert reason is None

    def test_ignores_abort_requested(self):
        inputs = MissionInputs(altitude_m=1.0, abort_requested=True)
        state, reason = next_state(MissionState.LAND, inputs, MissionConfig())
        assert state == MissionState.LAND
        assert reason is None

    def test_ignores_critical_battery(self):
        inputs = MissionInputs(altitude_m=1.0, critical_battery=True)
        state, reason = next_state(MissionState.LAND, inputs, MissionConfig())
        assert state == MissionState.LAND
        assert reason is None

    def test_ignores_start_requested(self):
        inputs = MissionInputs(altitude_m=1.0, start_requested=True)
        state, reason = next_state(MissionState.LAND, inputs, MissionConfig())
        assert state == MissionState.LAND
        assert reason is None


class TestPriorityOrdering:
    def test_takeoff_abort_beats_everything(self):
        inputs = MissionInputs(
            abort_requested=True,
            critical_battery=True,
            battery_pct=1.0,
            rtl_requested=True,
            altitude_m=100.0,
        )
        state, reason = next_state(MissionState.TAKEOFF, inputs, MissionConfig())
        assert state == MissionState.LAND
        assert reason == 'abort requested'

    def test_takeoff_critical_battery_beats_low_battery_and_rtl(self):
        inputs = MissionInputs(
            critical_battery=True,
            battery_pct=1.0,
            rtl_requested=True,
            altitude_m=100.0,
        )
        state, reason = next_state(MissionState.TAKEOFF, inputs, MissionConfig())
        assert state == MissionState.LAND
        assert reason == 'critical battery'

    def test_takeoff_low_battery_beats_rtl_requested_and_altitude(self):
        inputs = MissionInputs(
            battery_pct=1.0,
            rtl_requested=True,
            altitude_m=100.0,
        )
        state, reason = next_state(MissionState.TAKEOFF, inputs, MissionConfig())
        assert state == MissionState.RTL
        assert reason == 'low battery'

    def test_takeoff_rtl_requested_beats_altitude_reached(self):
        inputs = MissionInputs(rtl_requested=True, altitude_m=100.0)
        state, reason = next_state(MissionState.TAKEOFF, inputs, MissionConfig())
        assert state == MissionState.RTL
        assert reason == 'RTL requested'

    def test_mission_abort_beats_everything(self):
        inputs = MissionInputs(
            abort_requested=True,
            critical_battery=True,
            battery_pct=1.0,
            rtl_requested=True,
            mission_complete=True,
        )
        state, reason = next_state(MissionState.MISSION, inputs, MissionConfig())
        assert state == MissionState.LAND
        assert reason == 'abort requested'

    def test_mission_critical_battery_beats_low_battery_and_rtl(self):
        inputs = MissionInputs(
            critical_battery=True,
            battery_pct=1.0,
            rtl_requested=True,
            mission_complete=True,
        )
        state, reason = next_state(MissionState.MISSION, inputs, MissionConfig())
        assert state == MissionState.LAND
        assert reason == 'critical battery'

    def test_mission_low_battery_beats_rtl_requested_and_complete(self):
        inputs = MissionInputs(
            battery_pct=1.0,
            rtl_requested=True,
            mission_complete=True,
        )
        state, reason = next_state(MissionState.MISSION, inputs, MissionConfig())
        assert state == MissionState.RTL
        assert reason == 'low battery'

    def test_mission_rtl_requested_beats_mission_complete(self):
        inputs = MissionInputs(rtl_requested=True, mission_complete=True)
        state, reason = next_state(MissionState.MISSION, inputs, MissionConfig())
        assert state == MissionState.RTL
        assert reason == 'RTL requested'


class TestReasonStrings:
    def test_all_reason_strings_match_spec(self):
        config = MissionConfig()
        cases = [
            (MissionState.IDLE, MissionInputs(start_requested=True), 'start requested'),
            (MissionState.LANDED, MissionInputs(start_requested=True), 'start requested'),
            (MissionState.TAKEOFF, MissionInputs(abort_requested=True), 'abort requested'),
            (MissionState.TAKEOFF, MissionInputs(critical_battery=True), 'critical battery'),
            (
                MissionState.TAKEOFF,
                MissionInputs(battery_pct=config.rtl_battery_pct),
                'low battery',
            ),
            (MissionState.TAKEOFF, MissionInputs(rtl_requested=True), 'RTL requested'),
            (
                MissionState.TAKEOFF,
                MissionInputs(altitude_m=config.takeoff_altitude_m),
                'takeoff altitude reached',
            ),
            (MissionState.MISSION, MissionInputs(abort_requested=True), 'abort requested'),
            (MissionState.MISSION, MissionInputs(critical_battery=True), 'critical battery'),
            (
                MissionState.MISSION,
                MissionInputs(battery_pct=config.rtl_battery_pct),
                'low battery',
            ),
            (MissionState.MISSION, MissionInputs(rtl_requested=True), 'RTL requested'),
            (
                MissionState.MISSION,
                MissionInputs(mission_complete=True),
                'mission complete',
            ),
            (MissionState.RTL, MissionInputs(abort_requested=True), 'abort requested'),
            (MissionState.RTL, MissionInputs(critical_battery=True), 'critical battery'),
            (
                MissionState.RTL,
                MissionInputs(distance_to_target_m=config.arrival_tolerance_m),
                'home position reached',
            ),
            (
                MissionState.LAND,
                MissionInputs(altitude_m=config.landed_altitude_m),
                'touchdown',
            ),
        ]
        for state, inputs, expected_reason in cases:
            _, reason = next_state(state, inputs, config)
            assert reason == expected_reason

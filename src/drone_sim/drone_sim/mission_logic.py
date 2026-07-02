"""Pure mission state-machine logic for the drone simulation (no ROS imports)."""

from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple


class MissionState(Enum):
    """Flight mode of the mission state machine."""

    IDLE = 'IDLE'
    TAKEOFF = 'TAKEOFF'
    MISSION = 'MISSION'
    RTL = 'RTL'
    LAND = 'LAND'
    LANDED = 'LANDED'


@dataclass(frozen=True)
class MissionConfig:
    """Static thresholds used by the transition rules."""

    takeoff_altitude_m: float = 1.5
    rtl_battery_pct: float = 30.0
    arrival_tolerance_m: float = 0.25
    landed_altitude_m: float = 0.05


@dataclass(frozen=True)
class MissionInputs:
    """Snapshot of the world state evaluated on every tick."""

    altitude_m: float = 0.0
    battery_pct: float = 100.0
    distance_to_target_m: float = float('inf')
    mission_complete: bool = False
    start_requested: bool = False
    abort_requested: bool = False
    rtl_requested: bool = False
    critical_battery: bool = False


def next_state(
    state: MissionState,
    inputs: MissionInputs,
    config: MissionConfig,
) -> Tuple[MissionState, Optional[str]]:
    """Evaluate one state-machine tick and return (new_state, reason).

    Guards are evaluated in priority order for each state. When no guard
    fires, the state is unchanged and the reason is None.
    """
    if state == MissionState.IDLE:
        if inputs.start_requested:
            return MissionState.TAKEOFF, 'start requested'
        return state, None

    if state == MissionState.LANDED:
        if inputs.start_requested:
            return MissionState.TAKEOFF, 'start requested'
        return state, None

    if state in (MissionState.TAKEOFF, MissionState.MISSION, MissionState.RTL):
        if inputs.abort_requested:
            return MissionState.LAND, 'abort requested'
        if inputs.critical_battery:
            return MissionState.LAND, 'critical battery'
        if state in (MissionState.TAKEOFF, MissionState.MISSION):
            if inputs.battery_pct <= config.rtl_battery_pct:
                return MissionState.RTL, 'low battery'
            if inputs.rtl_requested:
                return MissionState.RTL, 'RTL requested'

        if state == MissionState.TAKEOFF:
            if inputs.altitude_m >= config.takeoff_altitude_m - config.arrival_tolerance_m:
                return MissionState.MISSION, 'takeoff altitude reached'
        elif state == MissionState.MISSION:
            if inputs.mission_complete:
                return MissionState.RTL, 'mission complete'
        elif state == MissionState.RTL:
            if inputs.distance_to_target_m <= config.arrival_tolerance_m:
                return MissionState.LAND, 'home position reached'

        return state, None

    if state == MissionState.LAND:
        if inputs.altitude_m <= config.landed_altitude_m:
            return MissionState.LANDED, 'touchdown'
        return state, None

    return state, None

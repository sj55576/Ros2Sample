"""Tests for emergency landing logic."""


def landing_command(is_landing: bool, altitude: float, descent_speed: float) -> float:
    """
    Replicate the _tick logic from EmergencyLand.

    Returns the commanded vertical velocity (negative = descending).
    Returns 0.0 if not landing or already landed.
    """
    if not is_landing:
        return 0.0
    if altitude > 0.05:
        return -descent_speed
    return 0.0


def should_trigger_landing(
    low_battery: bool,
    currently_landing: bool,
) -> bool:
    """Replicate _on_low_battery logic."""
    return low_battery and not currently_landing


class TestLandingCommand:

    def test_not_landing(self):
        assert landing_command(False, 10.0, 0.5) == 0.0

    def test_descending(self):
        assert landing_command(True, 5.0, 0.5) == -0.5

    def test_near_ground(self):
        assert landing_command(True, 0.03, 0.5) == 0.0

    def test_at_threshold(self):
        assert landing_command(True, 0.05, 0.5) == 0.0

    def test_just_above_threshold(self):
        assert landing_command(True, 0.06, 0.5) == -0.5

    def test_custom_descent_speed(self):
        assert landing_command(True, 10.0, 1.0) == -1.0


class TestLandingTrigger:

    def test_low_battery_triggers(self):
        assert should_trigger_landing(True, False) is True

    def test_normal_battery_no_trigger(self):
        assert should_trigger_landing(False, False) is False

    def test_already_landing_no_retrigger(self):
        assert should_trigger_landing(True, True) is False

    def test_normal_battery_while_landing(self):
        assert should_trigger_landing(False, True) is False

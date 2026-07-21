"""Tests for OpenUSD time and quaternion conversion helpers."""

import math

from openusd_bridge.sampling import (
    normalized_quaternion,
    relative_time_code,
    stamp_to_seconds,
)
import pytest


def test_stamp_to_seconds():
    """ROS seconds and nanoseconds are combined without integer truncation."""
    assert stamp_to_seconds(12, 500_000_000) == pytest.approx(12.5)


def test_stamp_rejects_invalid_nanoseconds():
    """Nanoseconds outside a ROS timestamp are rejected."""
    with pytest.raises(ValueError, match='invalid ROS timestamp'):
        stamp_to_seconds(1, 1_000_000_000)


def test_relative_time_code_uses_stage_rate():
    """One elapsed second maps to one stage-rate worth of time codes."""
    assert relative_time_code(11.0, 10.0, 24.0) == pytest.approx(24.0)


def test_relative_time_code_clamps_out_of_order_samples():
    """An old timestamp does not create a negative time code."""
    assert relative_time_code(9.0, 10.0, 30.0) == pytest.approx(0.0)


def test_normalized_quaternion():
    """Quaternion components are normalized before OpenUSD authoring."""
    assert normalized_quaternion(0.0, 0.0, 2.0, 2.0) == pytest.approx(
        (0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)),
    )


def test_zero_quaternion_becomes_identity():
    """An all-zero ROS quaternion becomes a valid identity quaternion."""
    assert normalized_quaternion(0.0, 0.0, 0.0, 0.0) == (
        0.0, 0.0, 0.0, 1.0,
    )

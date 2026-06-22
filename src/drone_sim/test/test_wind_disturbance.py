"""Tests for the drone_sim.wind_disturbance module."""

import math

import pytest

from drone_sim.wind_utils import compute_wind


def test_zero_base_zero_gust_zero_turbulence():
    """All-zero parameters produce a zero wind vector at any time."""
    wx, wy, wz = compute_wind(
        base=(0.0, 0.0, 0.0),
        gust_amplitude=0.0,
        gust_period_sec=8.0,
        turbulence_intensity=0.0,
        elapsed_sec=1.0,
    )
    assert wx == pytest.approx(0.0)
    assert wy == pytest.approx(0.0)
    assert wz == pytest.approx(0.0)


def test_base_wind_only():
    """With gust and turbulence disabled the result equals the base wind."""
    wx, wy, wz = compute_wind(
        base=(1.5, -0.5, 0.2),
        gust_amplitude=0.0,
        gust_period_sec=8.0,
        turbulence_intensity=0.0,
        elapsed_sec=3.0,
    )
    assert wx == pytest.approx(1.5)
    assert wy == pytest.approx(-0.5)
    assert wz == pytest.approx(0.2)


def test_gust_adds_to_base():
    """At t = gust_period/4 the X gust contribution equals +gust_amplitude."""
    period = 8.0
    amplitude = 0.3
    t = period / 4.0  # sin(2*pi*t/period) = sin(pi/2) = 1.0
    wx, wy, wz = compute_wind(
        base=(0.5, 0.0, 0.0),
        gust_amplitude=amplitude,
        gust_period_sec=period,
        turbulence_intensity=0.0,
        elapsed_sec=t,
    )
    assert wx == pytest.approx(0.5 + amplitude)


def test_gust_zero_at_start():
    """At t=0, sin(0)=0 so the gust contributes nothing to the wind vector."""
    wx, wy, wz = compute_wind(
        base=(0.5, 0.2, 0.0),
        gust_amplitude=1.0,
        gust_period_sec=8.0,
        turbulence_intensity=0.0,
        elapsed_sec=0.0,
    )
    assert wx == pytest.approx(0.5)
    assert wy == pytest.approx(0.2 + 1.0 * math.sin(math.pi / 2.0))
    assert wz == pytest.approx(0.0)


def test_turbulence_bounded():
    """The turbulence component magnitude never exceeds turbulence_intensity."""
    intensity = 0.1
    for t in [0.0, 0.1, 1.0, 5.0, 50.0, 123.456]:
        wx, wy, wz = compute_wind(
            base=(0.0, 0.0, 0.0),
            gust_amplitude=0.0,
            gust_period_sec=8.0,
            turbulence_intensity=intensity,
            elapsed_sec=t,
        )
        assert abs(wx) <= intensity + 1e-9
        assert abs(wy) <= intensity + 1e-9
        assert abs(wz) <= intensity + 1e-9


def test_deterministic():
    """The same inputs always produce the same output."""
    kwargs = dict(
        base=(0.5, 0.1, 0.0),
        gust_amplitude=0.3,
        gust_period_sec=8.0,
        turbulence_intensity=0.1,
        elapsed_sec=3.7,
    )
    first = compute_wind(**kwargs)
    second = compute_wind(**kwargs)
    assert first[0] == pytest.approx(second[0])
    assert first[1] == pytest.approx(second[1])
    assert first[2] == pytest.approx(second[2])


def test_wind_varies_over_time():
    """Wind vector at t=0 differs from wind vector at t=gust_period/4."""
    period = 8.0
    result_t0 = compute_wind(
        base=(0.5, 0.0, 0.0),
        gust_amplitude=0.3,
        gust_period_sec=period,
        turbulence_intensity=0.0,
        elapsed_sec=0.0,
    )
    result_tq = compute_wind(
        base=(0.5, 0.0, 0.0),
        gust_amplitude=0.3,
        gust_period_sec=period,
        turbulence_intensity=0.0,
        elapsed_sec=period / 4.0,
    )
    assert result_t0 != pytest.approx(result_tq)

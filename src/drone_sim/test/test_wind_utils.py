"""Tests for the drone_sim.wind_utils module."""

import math

from drone_sim.wind_utils import compute_wind
import pytest


def test_zero_everything_returns_base():
    """With no gust, no turbulence and zero elapsed time, wind equals base."""
    base = (1.0, 2.0, 3.0)
    result = compute_wind(base, gust_amplitude=0.0, gust_period_sec=1.0,
                          turbulence_intensity=0.0, elapsed_sec=0.0)
    assert result == pytest.approx(base)


def test_zero_gust_period_disables_gust():
    """A non-positive gust period disables the gust component entirely."""
    base = (0.0, 0.0, 0.0)
    result = compute_wind(base, gust_amplitude=5.0, gust_period_sec=0.0,
                          turbulence_intensity=0.0, elapsed_sec=1.0)
    assert result == pytest.approx((0.0, 0.0, 0.0))


def test_negative_gust_period_disables_gust():
    """A negative gust period also disables the gust component."""
    base = (0.0, 0.0, 0.0)
    result = compute_wind(base, gust_amplitude=5.0, gust_period_sec=-1.0,
                          turbulence_intensity=0.0, elapsed_sec=1.0)
    assert result == pytest.approx((0.0, 0.0, 0.0))


def test_gust_at_zero_elapsed_time_is_zero():
    """At elapsed_sec=0 the sine-based gust_x component is zero."""
    base = (0.0, 0.0, 0.0)
    result = compute_wind(base, gust_amplitude=3.0, gust_period_sec=4.0,
                          turbulence_intensity=0.0, elapsed_sec=0.0)
    assert result[0] == pytest.approx(0.0, abs=1e-9)


def test_gust_x_and_y_are_quarter_period_out_of_phase():
    """gust_y leads gust_x by a quarter period (matches the pi/2 phase offset)."""
    base = (0.0, 0.0, 0.0)
    amplitude = 2.0
    period = 8.0
    elapsed = period / 4.0
    result = compute_wind(base, gust_amplitude=amplitude, gust_period_sec=period,
                          turbulence_intensity=0.0, elapsed_sec=elapsed)
    # At a quarter period, gust_x = amplitude * sin(pi/2) = amplitude.
    assert result[0] == pytest.approx(amplitude)


def test_zero_turbulence_intensity_gives_no_turbulence_noise():
    """Zero turbulence intensity means the z component matches base exactly."""
    base = (0.0, 0.0, 7.0)
    result = compute_wind(base, gust_amplitude=0.0, gust_period_sec=1.0,
                          turbulence_intensity=0.0, elapsed_sec=42.0)
    assert result[2] == pytest.approx(7.0)


def test_turbulence_bounded_by_intensity():
    """Turbulence contribution never exceeds the given intensity in magnitude."""
    base = (0.0, 0.0, 0.0)
    intensity = 0.5
    for elapsed in (0.1, 1.0, 3.7, 10.0, 100.0):
        result = compute_wind(base, gust_amplitude=0.0, gust_period_sec=1.0,
                              turbulence_intensity=intensity, elapsed_sec=elapsed)
        for component in result:
            assert abs(component) <= intensity + 1e-9


def test_result_is_finite_for_large_elapsed_time():
    """Wind computation stays finite for a large elapsed time value."""
    base = (0.0, 0.0, 0.0)
    result = compute_wind(base, gust_amplitude=1.0, gust_period_sec=2.0,
                          turbulence_intensity=1.0, elapsed_sec=1.0e6)
    assert all(math.isfinite(component) for component in result)


def test_base_offset_is_additive():
    """A non-zero base is added on top of the gust and turbulence components."""
    zero_base_result = compute_wind(
        (0.0, 0.0, 0.0), gust_amplitude=1.0, gust_period_sec=2.0,
        turbulence_intensity=0.5, elapsed_sec=1.5,
    )
    offset_base = (10.0, -5.0, 2.0)
    offset_result = compute_wind(
        offset_base, gust_amplitude=1.0, gust_period_sec=2.0,
        turbulence_intensity=0.5, elapsed_sec=1.5,
    )
    for offset_component, zero_component, base_component in zip(
            offset_result, zero_base_result, offset_base):
        assert offset_component == pytest.approx(zero_component + base_component)

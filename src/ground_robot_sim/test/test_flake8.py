"""Flake8 linter test for the ground_robot_sim package."""

import pytest

ament_flake8_main = pytest.importorskip('ament_flake8.main')


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    """Enforce PEP8 / flake8 compliance across the ground_robot_sim package."""
    rc, errors = ament_flake8_main.main_with_errors(argv=[])
    assert rc == 0, f'Found {len(errors)} code style errors / warnings'

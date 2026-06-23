"""Pep257 docstring test for the sensor_fusion_sim package."""

import pytest

ament_pep257_main = pytest.importorskip('ament_pep257.main')


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    """Enforce pep257 docstring conventions."""
    rc = ament_pep257_main.main(argv=[])
    assert rc == 0, 'Found docstring style errors'

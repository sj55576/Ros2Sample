"""PEP257 linter test for the openusd_bridge package."""

import pytest

ament_pep257_main = pytest.importorskip('ament_pep257.main')


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    """Enforce docstring conventions across the package."""
    rc = ament_pep257_main.main(argv=['.'])
    assert rc == 0

"""PEP 257 docstring convention test for the ros2_learning package."""

import pytest

ament_pep257_main = pytest.importorskip('ament_pep257.main')


@pytest.mark.pep257
@pytest.mark.linter
def test_pep257():
    """Enforce PEP 257 docstring conventions for ros2_learning."""
    rc = ament_pep257_main.main(argv=['.', 'test'])
    assert rc == 0, 'Found PEP 257 docstring errors'

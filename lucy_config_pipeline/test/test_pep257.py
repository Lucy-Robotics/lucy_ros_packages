from pathlib import Path

from ament_pep257.main import main
import pytest


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    # Scope to this package only — CI coverage runs pytest from the workspace
    # root, so argv=['.', 'test'] would lint build/, install/, and siblings.
    pkg_root = Path(__file__).resolve().parents[1]
    rc = main(argv=[
        str(pkg_root / 'src'),
        str(pkg_root / 'launch'),
        str(pkg_root / 'setup.py'),
        str(pkg_root / 'test'),
    ])
    assert rc == 0, 'Found code style errors / warnings'

from pathlib import Path

from ament_pep257.main import main
import pytest


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    # Absolute, package-scoped paths: "." is relative to the *invoking*
    # process's cwd, not this file, so under a workspace-wide test runner
    # (e.g. a coverage step run from the workspace root) it would also
    # check every sibling package.
    pkg_root = Path(__file__).resolve().parents[1]
    rc = main(argv=[str(pkg_root)])
    assert rc == 0, "Found code style errors / warnings"

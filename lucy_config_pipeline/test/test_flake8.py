from pathlib import Path
import warnings

from ament_flake8.main import main_with_errors
import pytest


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    pkg_root = Path(__file__).resolve().parents[1]
    targets = [
        "--config",
        str(pkg_root / "ament_flake8.ini"),
        str(pkg_root / "setup.py"),
        str(pkg_root / "src"),
        str(pkg_root / "test"),
    ]
    with warnings.catch_warnings():
        warnings.filterwarnings(
            "ignore",
            message="SelectableGroups dict interface is deprecated",
            category=DeprecationWarning,
        )
        rc, errors = main_with_errors(argv=targets)
    assert rc == 0, "Found %d code style errors / warnings:\n" % len(
        errors
    ) + "\n".join(errors)

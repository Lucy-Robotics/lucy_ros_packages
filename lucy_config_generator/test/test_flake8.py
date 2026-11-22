# Copyright 2017 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
        str(pkg_root / "lucy_config_generator"),
        str(pkg_root / "test"),
    ]
    # flake8 uses importlib.metadata.entry_points().get(); Python 3.10+ deprecates it.
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

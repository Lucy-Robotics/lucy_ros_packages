# Copyright 2026 Lucy Robotics Team
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

"""Verify launch Python files compile (syntax only; no ROS runtime)."""
from pathlib import Path
import py_compile


def test_launch_py_files_compile():
    launch_dir = Path(__file__).resolve().parents[1] / 'launch'
    files = sorted(launch_dir.glob('*.py'))
    assert files, 'expected launch/*.py'
    for path in files:
        py_compile.compile(str(path), doraise=True)

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

"""Pytest configuration for camera_ros tests."""

from pathlib import Path
import sys

# Set up Python path at module level to ensure it's available before any imports
# This is needed when running tests directly (not through colcon test)
workspace_path = Path('/home/dev/lucy_ws')
install_path = workspace_path / 'install' / 'camera_ros'

if install_path.exists():
    python_version = f'{sys.version_info.major}.{sys.version_info.minor}'
    dist_packages = install_path / 'local' / 'lib' / f'python{python_version}' / 'dist-packages'

    if dist_packages.exists() and str(dist_packages) not in sys.path:
        sys.path.insert(0, str(dist_packages))

# Add scripts directory
package_path = Path(__file__).parent.parent
scripts_path = package_path / 'scripts'
if scripts_path.exists() and str(scripts_path) not in sys.path:
    sys.path.insert(0, str(scripts_path))

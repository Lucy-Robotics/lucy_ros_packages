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

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

from depth_colorizer import colorize  # noqa: E402


def test_near_is_blue_far_is_red_invalid_is_black():
    depth = np.array([[200, 3000, 0, 9000]], dtype=np.uint16)
    b, g, r = colorize(depth, 200, 3000)[0].T.astype(int)
    assert b[0] > r[0]  # near -> blue
    assert r[1] > b[1]  # far -> red
    assert (b[2], g[2], r[2]) == (0, 0, 0)  # no depth -> black
    assert (b[3], r[3]) == (b[1], r[1])  # beyond max clamps to far

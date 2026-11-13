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

"""web_ros_api.launch.py moves rosbridge to the next free port when its port is busy."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import socket

import pytest

_LAUNCH = Path(__file__).resolve().parents[1] / "launch" / "web_ros_api.launch.py"
ADDRESS = "127.0.0.1"


@pytest.fixture(scope="module")
def mod():
    spec = importlib.util.spec_from_file_location("lucy_bringup_web_ros_api", _LAUNCH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(module)
    except ModuleNotFoundError as exc:
        pytest.skip(f"ROS launch deps missing: {exc}")
    return module


@pytest.fixture
def listening():
    """Listen on a free local port, as another rosbridge would."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind((ADDRESS, 0))
    sock.listen()
    yield sock.getsockname()[1]
    sock.close()


def _selected(mod, **configs):
    from launch import LaunchContext

    context = LaunchContext()
    context.launch_configurations.update(configs)
    for action in mod._select_rosbridge_port(context):
        action.execute(context)
    return context.launch_configurations["rosbridge_selected_port"]


def test_busy_port_moves_to_the_next_free_one(mod, listening):
    port = mod.free_rosbridge_port(ADDRESS, listening)
    assert port > listening
    assert mod.port_is_free(ADDRESS, port)


def test_free_port_is_kept(mod, listening):
    assert not mod.port_is_free(ADDRESS, listening)
    port = mod.free_rosbridge_port(ADDRESS, listening + 1)
    assert mod.free_rosbridge_port(ADDRESS, port) == port


def test_no_free_port_in_span_raises(mod, listening):
    with pytest.raises(RuntimeError, match="no free rosbridge port"):
        mod.free_rosbridge_port(ADDRESS, listening, span=1)


def test_launch_sets_the_port_rosbridge_gets(mod, listening):
    selected = _selected(mod, rosbridge_port=str(listening), rosbridge_address=ADDRESS)
    assert int(selected) > listening


@pytest.mark.parametrize("value", ["abc", "0", "70000", ""])
def test_invalid_rosbridge_port_raises(mod, value):
    with pytest.raises(RuntimeError, match="must be a port number"):
        _selected(mod, rosbridge_port=value)

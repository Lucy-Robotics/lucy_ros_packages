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

"""Bringup helpers for one mcu_bridge per board with a serial."""

from __future__ import annotations

import importlib.util
from pathlib import Path
from unittest.mock import patch

import pytest
import yaml

_LAUNCH = Path(__file__).resolve().parents[1] / 'launch' / 'lucy.launch.py'


def _load_launch_module():
    spec = importlib.util.spec_from_file_location('lucy_bringup_lucy_launch', _LAUNCH)
    assert spec is not None and spec.loader is not None
    mod = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(mod)
    except ModuleNotFoundError as exc:
        pytest.skip(f'ROS launch deps missing: {exc}')
    return mod


def test_boards_with_serial_uses_derive_ros2_node_name(tmp_path: Path):
    module = _load_launch_module()
    robot = tmp_path / 'thais_urdf'
    hw = robot / 'config' / 'hardware'
    hw.mkdir(parents=True)
    (hw / 'active.yaml').write_text(
        yaml.safe_dump(
            {
                'boards': {
                    'rp2040_left_arm': {'serial_id': 'ABC123'},
                    'rp2040_torso_head': {'serial_id': ''},
                    'rp2040_right_arm': {'serial_id': 'DEF456'},
                }
            }
        ),
        encoding='utf-8',
    )
    with (
        patch.object(module, 'get_package_share_directory', return_value=str(tmp_path)),
        patch.object(module, '_infer_robot_source_root', return_value=robot),
    ):
        assert module._boards_with_serial('thais_urdf') == [
            ('left_arm', 'ABC123'),
            ('right_arm', 'DEF456'),
        ]

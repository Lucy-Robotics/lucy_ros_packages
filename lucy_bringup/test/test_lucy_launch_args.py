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

"""lucy.launch.py argument rules: mode / bridge / headless / domain_id (no ROS graph)."""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest

_LAUNCH = Path(__file__).resolve().parents[1] / "launch" / "lucy.launch.py"

DEFAULTS = {
    "mode": "mock",
    "bridge": "",
    "headless": "false",
    "rviz": "false",
    "domain_id": "0",
    "robot_package": "inmoov_urdf",
}


def _load_launch_module():
    spec = importlib.util.spec_from_file_location("lucy_bringup_lucy_launch", _LAUNCH)
    assert spec is not None and spec.loader is not None
    mod = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(mod)
    except ModuleNotFoundError as exc:
        pytest.skip(f"ROS launch deps missing: {exc}")
    return mod


@pytest.fixture(scope="module")
def mod():
    return _load_launch_module()


def _context(**overrides):
    from launch import LaunchContext

    context = LaunchContext()
    context.launch_configurations.update({**DEFAULTS, **overrides})
    return context


def _applied(mod, **overrides):
    """Run _apply_launch_settings' actions; return (launch configurations, env)."""
    context = _context(**overrides)
    for action in mod._apply_launch_settings(context):
        action.execute(context)
    return context.launch_configurations, context.environment


@pytest.mark.parametrize(
    "overrides, gazebo, mock, bridge",
    [
        ({}, "false", "true", ""),
        ({"mode": "sim", "headless": "true"}, "true", "false", ""),
        ({"mode": "real", "bridge": "linux"}, "false", "false", "linux"),
        ({"mode": "real", "bridge": "MCU_BRIDGE"}, "false", "false", "mcu_bridge"),
    ],
)
def test_mode_drives_gazebo_mock_and_bridge(mod, overrides, gazebo, mock, bridge):
    configs, env = _applied(mod, **overrides)
    assert configs["use_gazebo"] == gazebo
    assert configs["use_mock_hardware"] == mock
    assert env["LUCY_BRIDGE"] == bridge


def test_domain_id_is_exported(mod):
    _, env = _applied(mod, domain_id="07")
    assert env["ROS_DOMAIN_ID"] == "7"


@pytest.mark.parametrize(
    "overrides, message",
    [
        ({"mode": "real"}, "needs bridge:=linux"),
        ({"mode": "real", "bridge": "usb"}, "not bridge:='usb'"),
        ({"bridge": "linux"}, "only applies to mode:=real"),
        ({"mode": "sim", "bridge": "linux"}, "only applies to mode:=real"),
        ({"headless": "true"}, "headless:=true only applies to mode:=sim"),
        ({"mode": "sim", "headless": "maybe"}, "must be true or false"),
        ({"domain_id": "102"}, "from 0 to 101"),
        ({"domain_id": "-1"}, "from 0 to 101"),
        ({"domain_id": "abc"}, "from 0 to 101"),
        ({"robot_package": ""}, "no robot-description package"),
    ],
)
def test_invalid_arguments_raise(mod, overrides, message):
    with pytest.raises(RuntimeError, match=message):
        mod.launch_settings(_context(**overrides))


def test_mode_is_limited_to_its_choices(mod):
    """launch_settings trusts mode: its DeclareLaunchArgument rejects anything else."""
    from launch.actions import DeclareLaunchArgument

    declared = {
        entity.name: entity
        for entity in mod.generate_launch_description().entities
        if isinstance(entity, DeclareLaunchArgument)
    }
    assert declared["mode"].choices == list(mod.MODES)

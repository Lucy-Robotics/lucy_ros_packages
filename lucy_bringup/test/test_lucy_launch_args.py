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

"""lucy.launch.py argument rules: mode / headless / domain_id and the robot's bridge."""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest

_LAUNCH = Path(__file__).resolve().parents[1] / "launch" / "lucy.launch.py"

DEFAULTS = {
    "mode": "mock",
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


@pytest.fixture
def robot(mod, tmp_path, monkeypatch):
    monkeypatch.setattr(mod, "_robot_root", lambda _package: tmp_path)

    def declare(text: str) -> None:
        (tmp_path / "config").mkdir(exist_ok=True)
        (tmp_path / "config" / "control.launch.yaml").write_text(text)

    return declare


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
    "mode, gazebo, mock",
    [("mock", "false", "true"), ("sim", "true", "false")],
)
def test_mode_drives_gazebo_and_mock_without_a_bridge(mod, robot, mode, gazebo, mock):
    robot("bridge: linux\n")
    configs, env = _applied(mod, mode=mode)
    assert configs["use_gazebo"] == gazebo
    assert configs["use_mock_hardware"] == mock
    assert env["LUCY_BRIDGE"] == ""


@pytest.mark.parametrize(
    "declared, bridge",
    [("bridge: linux\n", "linux"), ("bridge: MCU_BRIDGE\n", "mcu_bridge")],
)
def test_real_hardware_runs_the_bridge_the_robot_package_names(
    mod, robot, declared, bridge
):
    robot("urdf_path: description/urdf/robot.urdf.xacro\n" + declared)
    configs, env = _applied(mod, mode="real")
    assert configs["use_gazebo"] == "false"
    assert configs["use_mock_hardware"] == "false"
    assert env["LUCY_BRIDGE"] == bridge


@pytest.mark.parametrize(
    "declared, message",
    [
        ("urdf_path: description/urdf/robot.urdf.xacro\n", "to name its bridge"),
        ("bridge: usb\n", "not 'usb'"),
    ],
)
def test_real_hardware_needs_the_robot_package_to_name_a_known_bridge(
    mod, robot, declared, message
):
    robot(declared)
    with pytest.raises(RuntimeError, match=message):
        mod.launch_settings(_context(mode="real"))


def test_a_robot_package_without_control_launch_yaml_cannot_run_real_hardware(
    mod, robot
):
    with pytest.raises(RuntimeError, match="to name its bridge"):
        mod.launch_settings(_context(mode="real"))


def test_bridge_argument_is_refused_with_where_it_moved(mod, robot):
    robot("bridge: linux\n")
    with pytest.raises(RuntimeError, match="config/control.launch.yaml"):
        mod.launch_settings(_context(mode="real", bridge="mcu_bridge"))


def test_domain_id_is_exported(mod):
    _, env = _applied(mod, domain_id="07")
    assert env["ROS_DOMAIN_ID"] == "7"


@pytest.mark.parametrize(
    "overrides, message",
    [
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

from pathlib import Path

from launch import LaunchContext
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import SetEnvironmentVariable
from launch.actions import TimerAction
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from lucy_control_supervisor import gazebo_launch as gl
import pytest


def _context(**configs: str) -> LaunchContext:
    context = LaunchContext()
    context.launch_configurations.update(configs)
    return context


def _env(context: LaunchContext, actions) -> dict:
    return {
        perform_substitutions(context, a.name): perform_substitutions(context, a.value)
        for a in actions
        if isinstance(a, SetEnvironmentVariable)
    }


def test_headless_gets_a_writable_runtime_dir_and_a_local_gz_ip(monkeypatch):
    monkeypatch.delenv("LUCY_GPU_MODE", raising=False)
    monkeypatch.delenv("GZ_IP", raising=False)
    context = _context(headless="true")
    env = _env(context, gl._gpu_env_actions(context))
    assert env["XDG_RUNTIME_DIR"] == "/tmp/runtime-root"
    assert env["GZ_IP"] == "127.0.0.1"


def test_jetson_uses_the_nvidia_egl_vendor_and_keeps_a_set_gz_ip(monkeypatch):
    monkeypatch.setenv("LUCY_GPU_MODE", "Jetson")
    monkeypatch.setenv("GZ_IP", "10.0.0.2")
    context = _context(headless="false")
    env = _env(context, gl._gpu_env_actions(context))
    assert env["__GLX_VENDOR_LIBRARY_NAME"] == "nvidia"
    assert env["__EGL_VENDOR_LIBRARY_FILENAMES"].endswith("10_nvidia.json")
    assert "GZ_IP" not in env


def test_gui_session_keeps_the_user_runtime_dir(monkeypatch):
    monkeypatch.delenv("LUCY_GPU_MODE", raising=False)
    context = _context(headless="no")
    assert "XDG_RUNTIME_DIR" not in _env(context, gl._gpu_env_actions(context))


def test_spawners_start_two_seconds_apart_broadcaster_first(tmp_path: Path):
    yaml_path = tmp_path / "controllers.yaml"
    yaml_path.write_text(
        "controller_manager:\n  ros__parameters:\n    update_rate: 100\n"
        "    arm_controller: {type: x}\n    joint_state_broadcaster: {type: y}\n",
        encoding="utf-8",
    )
    first, second = gl._spawner_actions_from_yaml(
        _context(controllers_yaml=str(yaml_path))
    )
    assert isinstance(first, Node)
    assert isinstance(second, TimerAction)
    assert second.period == 2.0


def _description(monkeypatch, tmp_path: Path, platform: str):
    share = tmp_path / "share"
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(gl, "get_package_share_directory", lambda pkg: str(share / pkg))
    monkeypatch.setattr(
        gl, "get_package_prefix", lambda pkg: str(tmp_path / "prefix" / pkg)
    )
    monkeypatch.setattr(gl.sys, "platform", platform)
    return gl.gazebo_launch_description(
        "robot", "robot.urdf.xacro", "rc.xacro", "robot"
    )


def _gazebo_actions(description, headless: str):
    gazebo = [e for e in description.entities if isinstance(e, OpaqueFunction)][-1]
    return gazebo.execute(_context(headless=headless))


def _gz_args(actions) -> str:
    include = next(a for a in actions if isinstance(a, IncludeLaunchDescription))
    return dict(include.launch_arguments)["gz_args"]


@pytest.mark.parametrize(
    "platform,headless,flags,gui",
    [
        ("linux", "true", "-s -r --headless-rendering", False),
        ("linux", "false", "-r", False),
        ("darwin", "false", "-s -r", True),
    ],
)
def test_gazebo_runs_server_only_when_headless_or_on_macos(
    monkeypatch, tmp_path, platform, headless, flags, gui
):
    actions = _gazebo_actions(_description(monkeypatch, tmp_path, platform), headless)
    world = str(tmp_path / "share" / "robot" / "worlds" / "default.sdf")
    assert _gz_args(actions) == f"{flags} {world}"
    timers = [a for a in actions if isinstance(a, TimerAction)]
    assert bool(timers) == gui
    if gui:
        assert isinstance(timers[0].actions[0], ExecuteProcess)


def test_missing_ros_gz_sim_says_what_to_install(monkeypatch, tmp_path):
    def share(pkg):
        if pkg == "ros_gz_sim":
            raise LookupError(pkg)
        return str(tmp_path / pkg)

    monkeypatch.setattr(gl, "get_package_share_directory", share)
    with pytest.raises(RuntimeError, match="ros-jazzy-ros-gz-sim"):
        gl.gazebo_launch_description("robot", "r.xacro", "rc.xacro", "robot")

import os
from pathlib import Path
import tempfile

import rclpy
from src.services import robot_select_node as rs


def _package(prefix: Path, name: str, *, launch: bool, description: bool) -> None:
    share = prefix / "share" / name
    (share / "launch").mkdir(parents=True)
    if launch:
        (share / "launch" / "control.launch.py").write_text("", encoding="utf-8")
    if description:
        (share / "description").mkdir()


def test_only_packages_with_a_control_launch_and_a_description_are_robots(
    tmp_path, monkeypatch
):
    _package(tmp_path, "so_arm101_urdf", launch=True, description=True)
    _package(tmp_path, "inmoov_urdf", launch=True, description=True)
    _package(tmp_path, "lucy_bringup", launch=True, description=False)
    _package(tmp_path, "meshes_only", launch=False, description=True)
    names = ["so_arm101_urdf", "lucy_bringup", "inmoov_urdf", "meshes_only"]
    monkeypatch.setattr(
        rs, "get_packages_with_prefixes", lambda: {n: str(tmp_path) for n in names}
    )
    assert rs.installed_robot_packages() == ["inmoov_urdf", "so_arm101_urdf"]


def test_default_files_are_the_ones_the_launcher_watches(monkeypatch):
    monkeypatch.delenv("LUCY_SELECT_ROBOT_FILE", raising=False)
    monkeypatch.delenv("LUCY_STACK_STATUS_FILE", raising=False)
    uid = os.getuid()
    assert (
        rs.select_request_file()
        == Path(tempfile.gettempdir()) / f"lucy_select_robot.{uid}"
    )
    assert (
        rs.stack_status_file()
        == Path(tempfile.gettempdir()) / f"lucy_stack_status.{uid}"
    )


def test_ctrl_c_stops_the_node_quietly_and_shuts_rclpy_down(monkeypatch):
    destroyed = []

    class FakeNode:
        def destroy_node(self):
            destroyed.append(True)

    def interrupted_spin(node):
        raise KeyboardInterrupt

    monkeypatch.setattr(rs, "RobotSelectNode", FakeNode)
    monkeypatch.setattr(rs.rclpy, "spin", interrupted_spin)
    rs.main()
    assert destroyed == [True]
    assert not rclpy.ok()

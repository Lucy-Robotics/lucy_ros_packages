"""torque_enable forward controllers are spawned on real hardware only."""

from pathlib import Path
import time

from lucy_control_supervisor.controllers_spawn import controllers_to_spawn
from lucy_control_supervisor.controllers_spawn import torque_command_values
from lucy_control_supervisor.controllers_spawn import torque_enable_controllers
from lucy_control_supervisor.controllers_spawn import torque_publishers_matched
from lucy_control_supervisor.controllers_spawn import wait_for_torque_match
import pytest
import yaml

_PKG = Path(__file__).resolve().parents[1] / "lucy_control_supervisor"
SRC = (_PKG / "supervisor_node.py").read_text(encoding="utf-8")
SPAWN_SRC = (_PKG / "controllers_spawn.py").read_text(encoding="utf-8")

_FIXTURE = """
controller_manager:
  ros__parameters:
    update_rate: 100
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    so_arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController
    so_arm_torque_enable_controller:
      type: forward_command_controller/ForwardCommandController
joint_state_broadcaster:
  ros__parameters: {}
so_arm_controller:
  ros__parameters:
    joints: [Rotation, Pitch]
    command_interfaces: [position]
    state_interfaces: [position]
so_arm_torque_enable_controller:
  ros__parameters:
    joints: [Rotation, Pitch, Elbow, Wrist_Pitch, Wrist_Roll, Jaw]
    interface_name: torque_enable
"""


class _FakePub:
    def __init__(self, count: int = 0) -> None:
        self._count = count
        self.published: list[list[float]] = []

    def get_subscription_count(self) -> int:
        return self._count

    def publish(self, msg) -> None:
        self.published.append(list(msg.data))


def _write(tmp_path: Path) -> Path:
    path = tmp_path / "controllers.yaml"
    path.write_text(_FIXTURE, encoding="utf-8")
    loaded = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert loaded["so_arm_torque_enable_controller"]["ros__parameters"][
        "interface_name"
    ] == ("torque_enable")
    return path


def _wait_for(predicate, timeout_s, poll_s=0.01):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(poll_s)
    return predicate()


def test_gazebo_spawn_skips_torque_enable(tmp_path: Path):
    path = _write(tmp_path)
    assert controllers_to_spawn(path) == [
        "joint_state_broadcaster",
        "so_arm_controller",
    ]


def test_real_spawn_includes_torque_enable(tmp_path: Path):
    path = _write(tmp_path)
    assert controllers_to_spawn(path, include_torque_enable=True) == [
        "joint_state_broadcaster",
        "so_arm_controller",
        "so_arm_torque_enable_controller",
    ]
    assert torque_enable_controllers(path) == [("so_arm_torque_enable_controller", 6)]


def test_torque_command_is_one_value_per_joint():
    assert torque_command_values(False, 6) == [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    assert torque_command_values(True, 2) == [1.0, 1.0]


def test_post_spawn_waits_for_subscription_count():
    """Reload must not publish until DDS has matched the forward controller."""
    assert "_publish_torque_after_match" in SRC
    assert "wait_for_torque_match" in SRC
    assert "get_subscription_count" in SPAWN_SRC
    assert "TORQUE_MATCH_TIMEOUT_S" in SRC
    spawn = SRC.index("def _spawn_controllers")
    next_method = SRC.index("def _on_active_client")
    body = SRC[spawn:next_method]
    assert "_publish_torque_after_match" in body
    assert "self._publish_torque_commands()" not in body


def test_wait_publishes_after_match():
    """Stored ON value is published once every publisher reports a subscriber."""
    pub = _FakePub(count=0)
    pubs = {"so_arm_torque_enable_controller": (pub, 2)}
    assert not torque_publishers_matched(pubs)

    def appear_then_wait(predicate, timeout_s):
        started = time.time()

        def with_appear():
            if time.time() - started > 0.05:
                pub._count = 1
            return predicate()

        return _wait_for(with_appear, timeout_s)

    assert wait_for_torque_match(pubs, appear_then_wait, 1.0)
    assert torque_publishers_matched(pubs)
    # Mimic supervisor: publish stored ON after match.
    pub.publish(type("M", (), {"data": torque_command_values(True, 2)})())
    assert pub.published == [[1.0, 1.0]]


def test_wait_times_out_and_still_allows_publish():
    """Unmatched publishers return False; caller still publishes once."""
    pub = _FakePub(count=0)
    pubs = {"so_arm_torque_enable_controller": (pub, 6)}
    started = time.time()
    assert wait_for_torque_match(pubs, _wait_for, 0.15) is False
    assert time.time() - started < 1.0
    pub.publish(type("M", (), {"data": torque_command_values(True, 6)})())
    assert pub.published == [[1.0, 1.0, 1.0, 1.0, 1.0, 1.0]]


def test_mcu_boards_follow_the_active_yaml(tmp_path: Path):
    from lucy_control_supervisor.bridges import mcu_boards

    hw = tmp_path / "active.yaml"
    hw.write_text(
        "boards:\n"
        '  mega_left: {board_class: i2c_pwm_only, serial_id: "AAA"}\n'
        '  mega_right: {board_class: i2c_pwm_only, serial_id: ""}\n'
        '  rp2040_torso_head: {board_class: internal_servo_only, serial_id: "BBB"}\n',
        encoding="utf-8",
    )
    assert mcu_boards(hw) == [("left", "AAA"), ("right", ""), ("torso_head", "BBB")]
    assert mcu_boards(tmp_path / "missing.yaml") == []


def test_ros2_control_nodes_reads_the_hardware_blocks():
    from lucy_control_supervisor.bridges import ros2_control_nodes

    urdf = (
        '<robot name="r">'
        '<ros2_control name="left" type="system"><hardware>'
        '<plugin>lucy_ros2_control/LucySystem</plugin><param name="node_name">left</param>'
        "</hardware></ros2_control>"
        '<ros2_control name="mock" type="system"><hardware><plugin>mock</plugin></hardware>'
        "</ros2_control>"
        "</robot>"
    )
    assert ros2_control_nodes(urdf) == {"left"}


_TWO_NODE_URDF = (
    '<robot name="r">'
    '<ros2_control name="a" type="system"><hardware>'
    '<param name="node_name">left</param></hardware></ros2_control>'
    '<ros2_control name="b" type="system"><hardware>'
    '<param name="node_name">so_arm</param></hardware></ros2_control>'
    "</robot>"
)


def test_linux_bridge_runs_one_firmware_per_ros2_control_node(tmp_path: Path):
    from lucy_control_supervisor.bridges import bridge_processes

    assert bridge_processes("linux", tmp_path / "missing.yaml", _TWO_NODE_URDF) == [
        ("left", ["left"]),
        ("so_arm", ["so_arm"]),
    ]


def test_mcu_bridge_runs_one_bridge_per_board_the_urdf_drives(tmp_path: Path):
    from lucy_control_supervisor.bridges import bridge_processes

    hw = tmp_path / "active.yaml"
    hw.write_text(
        "boards:\n"
        '  mega_left: {board_class: i2c_pwm_only, serial_id: "AAA"}\n'
        '  mega_right: {board_class: i2c_pwm_only, serial_id: "CCC"}\n',
        encoding="utf-8",
    )
    assert bridge_processes("mcu_bridge", hw, _TWO_NODE_URDF) == [("left", ["left", "AAA"])]
    assert bridge_processes("mcu_bridge", tmp_path / "missing.yaml", _TWO_NODE_URDF) == []


def test_mcu_bridge_finds_a_single_board_without_serial(tmp_path: Path):
    """SO-ARM101: one RP2040 and no serial_id, so the bridge takes the board it finds."""
    from lucy_control_supervisor.bridges import bridge_processes

    hw = tmp_path / "active.yaml"
    hw.write_text(
        'boards:\n  rp2040_so_arm: {board_class: bus_servo_only, serial_id: ""}\n',
        encoding="utf-8",
    )
    assert bridge_processes("mcu_bridge", hw, _TWO_NODE_URDF) == [("so_arm", ["so_arm"])]


def test_mcu_bridge_needs_serials_when_several_boards(tmp_path: Path):
    """Two bridges without a serial_id would both take the first board they see."""
    from lucy_control_supervisor.bridges import bridge_processes

    hw = tmp_path / "active.yaml"
    hw.write_text(
        "boards:\n"
        '  mega_left: {board_class: i2c_pwm_only, serial_id: "AAA"}\n'
        '  rp2040_so_arm: {board_class: bus_servo_only, serial_id: ""}\n',
        encoding="utf-8",
    )
    assert bridge_processes("mcu_bridge", hw, _TWO_NODE_URDF) == [("left", ["left", "AAA"])]


def test_unknown_bridge_is_rejected(tmp_path: Path):
    from lucy_control_supervisor.bridges import bridge_processes

    with pytest.raises(ValueError, match="unknown bridge"):
        bridge_processes("usb", tmp_path / "active.yaml", _TWO_NODE_URDF)


def test_bridge_executable_names_the_missing_package(monkeypatch):
    pytest.importorskip("ros2run")
    import ros2run.api

    from lucy_control_supervisor import bridges

    monkeypatch.setattr(ros2run.api, "get_executable_path", lambda **_k: None)
    with pytest.raises(LookupError, match="lucy_embedded_firmware_linux is not built"):
        bridges.bridge_executable("linux")

import json

import pytest
import rclpy
from src.services.robot_select_node import RobotSelectNode
from std_msgs.msg import String


@pytest.fixture
def node(tmp_path, monkeypatch):
    monkeypatch.setenv("LUCY_SELECT_ROBOT_FILE", str(tmp_path / "select"))
    monkeypatch.setenv("LUCY_STACK_STATUS_FILE", str(tmp_path / "status"))
    rclpy.init()
    n = RobotSelectNode(available=["inmoov_urdf", "so_arm101_urdf"])
    yield n
    n.destroy_node()
    rclpy.shutdown()


def test_installed_robot_switch_is_forwarded_to_the_launcher(node, tmp_path):
    node._on_select(String(data="so_arm101_urdf"))
    assert (tmp_path / "select").read_text() == "so_arm101_urdf"


def test_unknown_or_active_robot_is_ignored(node, tmp_path):
    node._active = "inmoov_urdf"
    node._on_select(String(data="../../etc"))
    node._on_select(String(data="inmoov_urdf"))
    assert not (tmp_path / "select").exists()


def test_robot_list_is_published(node):
    received = []
    sub = node.create_subscription(
        String,
        "/lucy/robots",
        lambda m: received.append(m.data),
        node._robots_pub.qos_profile,
    )
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.05)
        if received:
            break
    node.destroy_subscription(sub)
    assert json.loads(received[0])["available"] == ["inmoov_urdf", "so_arm101_urdf"]


def test_robots_and_stack_status_are_republished_for_late_subscribers(node, tmp_path):
    robots, status = [], []
    node._robots_pub.publish = lambda m: robots.append(m.data)
    node._status_pub.publish = lambda m: status.append(m.data)
    node._republish()
    (tmp_path / "status").write_text('{"ready": false, "index": 6}')
    node._republish()
    node._republish()
    assert status == ['{"ready": false, "index": 6}'] * 2
    assert len(robots) == 3
    assert json.loads(robots[0])["available"] == ["inmoov_urdf", "so_arm101_urdf"]


def test_an_empty_stack_status_file_is_not_relayed(node, tmp_path):
    status = []
    node._status_pub.publish = lambda m: status.append(m.data)
    (tmp_path / "status").write_text("")
    node._republish()
    assert status == []

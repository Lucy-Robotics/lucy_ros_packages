import json

import pytest
import rclpy
from src.services.robot_select_node import RobotSelectNode
from std_msgs.msg import String


@pytest.fixture
def node(tmp_path, monkeypatch):
    monkeypatch.setenv('LUCY_SELECT_ROBOT_FILE', str(tmp_path / 'select'))
    rclpy.init()
    n = RobotSelectNode(available=['inmoov_urdf', 'so_arm101_urdf'])
    yield n
    n.destroy_node()
    rclpy.shutdown()


def test_installed_robot_switch_is_forwarded_to_the_launcher(node, tmp_path):
    node._on_select(String(data='so_arm101_urdf'))
    assert (tmp_path / 'select').read_text() == 'so_arm101_urdf'


def test_unknown_or_active_robot_is_ignored(node, tmp_path):
    node._active = 'inmoov_urdf'
    node._on_select(String(data='../../etc'))
    node._on_select(String(data='inmoov_urdf'))
    assert not (tmp_path / 'select').exists()


def test_robot_list_is_published(node):
    received = []
    sub = node.create_subscription(
        String, '/lucy/robots', lambda m: received.append(m.data), node._robots_pub.qos_profile
    )
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.05)
        if received:
            break
    node.destroy_subscription(sub)
    assert json.loads(received[0])['available'] == ['inmoov_urdf', 'so_arm101_urdf']

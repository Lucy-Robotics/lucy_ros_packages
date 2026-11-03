"""
Robot package selection for the web panel.

Publishes the active and installed robot packages, and asks the Lucy launcher
to restart the stack on another one, exactly like ticking it in the TUI.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
import tempfile

from ament_index_python.packages import get_packages_with_prefixes
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from std_msgs.msg import String

ROBOTS_TOPIC = '/lucy/robots'
SELECT_TOPIC = '/lucy/select_robot'

_LATCHED_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


def installed_robot_packages() -> list[str]:
    """Return the built packages that look like Lucy robots."""
    found = []
    for pkg, prefix in get_packages_with_prefixes().items():
        share = Path(prefix) / 'share' / pkg
        if (share / 'launch' / 'control.launch.py').is_file() and (share / 'description').is_dir():
            found.append(pkg)
    return sorted(found)


def select_request_file() -> Path:
    """Return the file the Lucy launcher watches for a robot change."""
    override = os.environ.get('LUCY_SELECT_ROBOT_FILE')
    if override:
        return Path(override)
    uid = getattr(os, 'getuid', lambda: 0)()
    return Path(tempfile.gettempdir()) / f'lucy_select_robot.{uid}'


class RobotSelectNode(Node):
    """Expose the robot list and forward a selection to the launcher."""

    def __init__(self, available: list[str] | None = None) -> None:
        super().__init__('lucy_robot_select')
        self.declare_parameter('robot_package', '')
        self._active = str(self.get_parameter('robot_package').value)
        self._available = installed_robot_packages() if available is None else available
        self._robots_pub = self.create_publisher(String, ROBOTS_TOPIC, _LATCHED_QOS)
        self.create_subscription(String, SELECT_TOPIC, self._on_select, 10)
        self._robots_pub.publish(
            String(data=json.dumps({'active': self._active, 'available': self._available}))
        )

    def _on_select(self, msg: String) -> None:
        robot = msg.data.strip()
        if robot not in self._available:
            self.get_logger().warning(f'ignoring unknown robot package {robot!r}')
            return
        if robot == self._active:
            return
        select_request_file().write_text(robot, encoding='utf-8')
        self.get_logger().info(f'asked the launcher to switch to {robot}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RobotSelectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

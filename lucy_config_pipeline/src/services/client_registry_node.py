"""
Transport-agnostic client presence and control arbitration.

See lucy_cli/developer.md for the protocol. Clients register by heart-beating
their id; this node is the single writer of the connected-client count and the
active controller, so the web panel (rosbridge) and the CLI (DDS) behave alike.
"""

from __future__ import annotations

import threading
import time

from lucy_msgs.srv import ClientControl
from lucy_msgs.srv import GetInt
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from std_msgs.msg import Int32
from std_msgs.msg import String

HEARTBEAT_TOPIC = '/lucy/client_heartbeat'
CLIENT_COUNT_TOPIC = '/lucy/client_count'
ACTIVE_CLIENT_TOPIC = '/lucy/active_client'
GET_CLIENT_COUNT_SERVICE = '/lucy/get_client_count'
CONTROL_SERVICE = '/lucy/control'

TICK_PERIOD_S = 1.0
CLIENT_TTL_S = 3.0  # ~3 missed beats at the 1 Hz client cadence

_LATCHED_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)
# Volatile keeps us compatible with rosbridge's default publisher QoS.
_HEARTBEAT_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
)


class ClientRegistryNode(Node):
    """Single-writer registry for connected clients and the active controller."""

    def __init__(self) -> None:
        super().__init__('lucy_client_registry')

        # client_id -> last-seen monotonic time.
        # Guarded by _state_lock: callbacks may run on different executor threads.
        # (Not _clients — Node uses that.)
        self._last_seen: dict[str, float] = {}
        self._active_client: str = ''
        self._state_lock = threading.Lock()

        self._count_pub = self.create_publisher(Int32, CLIENT_COUNT_TOPIC, _LATCHED_QOS)
        self._active_pub = self.create_publisher(String, ACTIVE_CLIENT_TOPIC, _LATCHED_QOS)

        self.create_subscription(String, HEARTBEAT_TOPIC, self._on_heartbeat, _HEARTBEAT_QOS)
        self.create_service(GetInt, GET_CLIENT_COUNT_SERVICE, self._on_get_client_count)
        self.create_service(ClientControl, CONTROL_SERVICE, self._on_control)

        self._tick_timer = self.create_timer(TICK_PERIOD_S, self._on_tick)

        self._publish_count()
        self._publish_active()
        self.get_logger().info('Client registry started')

    def _on_heartbeat(self, msg: String) -> None:
        client_id = msg.data
        if not client_id:
            return
        with self._state_lock:
            is_new = client_id not in self._last_seen
            self._last_seen[client_id] = time.monotonic()
            count = len(self._last_seen)
        if is_new:
            self.get_logger().info(f'Client registered: {client_id} ({count} connected)')
            self._publish_count()

    def _on_tick(self) -> None:
        deadline = time.monotonic() - CLIENT_TTL_S
        with self._state_lock:
            expired = [cid for cid, seen in self._last_seen.items() if seen < deadline]
            for cid in expired:
                del self._last_seen[cid]
            controller_lost = (
                bool(self._active_client)
                and self._active_client not in self._last_seen
            )
            if controller_lost:
                self._active_client = ''
            count = len(self._last_seen)
        for cid in expired:
            self.get_logger().info(f'Client expired: {cid} ({count} connected)')
        if controller_lost:
            self.get_logger().info('Active controller expired; control released')
        # Republish so volatile subscribers (web via rosbridge) converge without
        # a dedicated query; subscribers dedupe by value.
        self._publish_count()
        self._publish_active()

    def _on_get_client_count(
        self, _req: GetInt.Request, res: GetInt.Response
    ) -> GetInt.Response:
        with self._state_lock:
            res.value = len(self._last_seen)
        return res

    def _on_control(
        self, req: ClientControl.Request, res: ClientControl.Response
    ) -> ClientControl.Response:
        if not req.client_id:
            with self._state_lock:
                res.active_client = self._active_client
            res.success = False
            res.message = 'empty client_id'
            return res
        with self._state_lock:
            is_new = req.client_id not in self._last_seen  # a request also proves liveness
            self._last_seen[req.client_id] = time.monotonic()
            if req.acquire:
                self._active_client = req.client_id
            elif self._active_client == req.client_id:
                self._active_client = ''
            res.active_client = self._active_client
        if is_new:
            self._publish_count()
        self.get_logger().info(
            f"Control {'granted to' if req.acquire else 'released by'} {req.client_id}"
        )
        self._publish_active()
        res.success = True
        res.message = 'ok'
        return res

    def _publish_count(self) -> None:
        with self._state_lock:
            count = len(self._last_seen)
        self._count_pub.publish(Int32(data=count))

    def _publish_active(self) -> None:
        with self._state_lock:
            active = self._active_client
        self._active_pub.publish(String(data=active))


def main() -> None:  # pragma: no cover
    rclpy.init()
    node = ClientRegistryNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':  # pragma: no cover
    main()

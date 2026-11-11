"""
Transport-agnostic client presence and control arbitration.

See lucy_cli/developer.md for the protocol. Clients register by heart-beating
their id; this node is the single writer of the connected-client count and the
active controller, so the web panel (rosbridge) and the CLI (DDS) behave alike.
"""

from __future__ import annotations

from collections.abc import Callable
import threading
import time

from control_msgs.msg import DynamicJointState
from lucy_config_generator.motors import motor_values_by_joint
from lucy_msgs.srv import ClientControl
from lucy_msgs.srv import ControlLock
from lucy_msgs.srv import GetInt
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from std_msgs.msg import Int32
from std_msgs.msg import String
import yaml

HEARTBEAT_TOPIC = "/lucy/client_heartbeat"
GOODBYE_TOPIC = "/lucy/client_goodbye"
BRIDGE_CLIENT_COUNT_TOPIC = "/client_count"
CLIENT_COUNT_TOPIC = "/lucy/client_count"
ACTIVE_CLIENT_TOPIC = "/lucy/active_client"
GET_CLIENT_COUNT_SERVICE = "/lucy/get_client_count"
CONTROL_SERVICE = "/lucy/control"
DYNAMIC_JOINT_STATES_TOPIC = "/dynamic_joint_states"
NO_SERVO_POWER = "servos are not powered, connect the power supply"
CONTROL_LOCK_SERVICE = "/lucy/control_lock"

TICK_PERIOD_S = 1.0
CLIENT_TTL_S = 10.0  # ~10 missed beats at the 1 Hz client cadence
SOCKET_DROP_GRACE_S = 1.5

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

    def __init__(self, read_active_config: Callable[[], str] | None = None) -> None:
        super().__init__("lucy_client_registry")

        # client_id -> last-seen monotonic time.
        # Guarded by _state_lock: callbacks may run on different executor threads.
        # (Not _clients — Node uses that.)
        self._last_seen: dict[str, float] = {}
        self._active_client: str = ""
        # While set, nobody can take control: torque must stay off, e.g. with the
        # arm in someone's hands during calibration.
        self._lock_owner: str = ""
        self._bridge_clients: int | None = None
        self._servo_powered = True
        self._read_active_config = read_active_config
        self._min_volts: dict[str, float] = {}
        self._motor_specs_error = ""
        self._state_lock = threading.Lock()
        self._reload_min_volts()

        self._count_pub = self.create_publisher(Int32, CLIENT_COUNT_TOPIC, _LATCHED_QOS)
        self._active_pub = self.create_publisher(
            String, ACTIVE_CLIENT_TOPIC, _LATCHED_QOS
        )

        self.create_subscription(
            String, HEARTBEAT_TOPIC, self._on_heartbeat, _HEARTBEAT_QOS
        )
        self.create_subscription(
            String, GOODBYE_TOPIC, self._on_goodbye, _HEARTBEAT_QOS
        )
        self.create_subscription(
            Int32, BRIDGE_CLIENT_COUNT_TOPIC, self._on_bridge_client_count, _LATCHED_QOS
        )
        self.create_subscription(
            DynamicJointState,
            DYNAMIC_JOINT_STATES_TOPIC,
            self._on_dynamic_joint_states,
            10,
        )
        self.create_service(GetInt, GET_CLIENT_COUNT_SERVICE, self._on_get_client_count)
        self.create_service(ClientControl, CONTROL_SERVICE, self._on_control)
        self.create_service(ControlLock, CONTROL_LOCK_SERVICE, self._on_control_lock)

        self._tick_timer = self.create_timer(TICK_PERIOD_S, self._on_tick)

        self._publish_count()
        self._publish_active()
        self.get_logger().info("Client registry started")

    def _on_heartbeat(self, msg: String) -> None:
        client_id = msg.data
        if not client_id:
            return
        with self._state_lock:
            is_new = client_id not in self._last_seen
            self._last_seen[client_id] = time.monotonic()
            count = len(self._last_seen)
        if is_new:
            self.get_logger().info(
                f"Client registered: {client_id} ({count} connected)"
            )
            self._publish_count()

    def _on_goodbye(self, msg: String) -> None:
        client_id = msg.data
        with self._state_lock:
            if self._last_seen.pop(client_id, None) is None:
                return
            controller_lost = self._active_client == client_id
            if controller_lost:
                self._active_client = ""
            count = len(self._last_seen)
        self.get_logger().info(f"Client left: {client_id} ({count} connected)")
        if controller_lost:
            self.get_logger().info("Active controller left; control released")
        self._publish_count()
        self._publish_active()

    def _on_bridge_client_count(self, msg: Int32) -> None:
        dropped = self._bridge_clients is not None and msg.data < self._bridge_clients
        self._bridge_clients = msg.data
        if not dropped:
            return
        dropped_at = time.monotonic()
        timer = None

        def check() -> None:
            timer.cancel()
            self.destroy_timer(timer)
            if self._drop_silent_since(dropped_at, "disconnected"):
                self._publish_count()
                self._publish_active()

        timer = self.create_timer(SOCKET_DROP_GRACE_S, check)

    def _drop_silent_since(self, cutoff: float, reason: str) -> list[str]:
        with self._state_lock:
            gone = [cid for cid, seen in self._last_seen.items() if seen < cutoff]
            for cid in gone:
                del self._last_seen[cid]
            controller_lost = (
                bool(self._active_client) and self._active_client not in self._last_seen
            )
            if controller_lost:
                self._active_client = ""
            count = len(self._last_seen)
        for cid in gone:
            self.get_logger().info(f"Client {reason}: {cid} ({count} connected)")
        if controller_lost:
            self.get_logger().info(f"Active controller {reason}; control released")
        return gone

    def _reload_min_volts(self) -> None:
        if self._read_active_config is None:
            return
        try:
            config = yaml.safe_load(self._read_active_config()) or {}
            min_volts, error = motor_values_by_joint(config, "min_volts"), ""
        except (OSError, KeyError, yaml.YAMLError) as e:
            min_volts, error = {}, str(e)
        if error and error != self._motor_specs_error:
            self.get_logger().warning(
                f"Motor specs unavailable, no power check: {error}"
            )
        self._motor_specs_error = error
        with self._state_lock:
            self._min_volts = min_volts

    def _on_dynamic_joint_states(self, msg: DynamicJointState) -> None:
        volts = {
            joint: value
            for joint, interfaces in zip(msg.joint_names, msg.interface_values)
            for name, value in zip(interfaces.interface_names, interfaces.values)
            if name == "voltage"
        }
        with self._state_lock:
            checked = [joint for joint in volts if joint in self._min_volts]
            if not checked:
                return
            self._servo_powered = all(
                volts[joint] >= self._min_volts[joint] for joint in checked
            )
            controller_lost = not self._servo_powered and bool(self._active_client)
            if controller_lost:
                self._active_client = ""
        if controller_lost:
            self.get_logger().warning("Servo power lost; control released")
            self._publish_active()

    def _on_tick(self) -> None:
        self._reload_min_volts()
        self._drop_silent_since(time.monotonic() - CLIENT_TTL_S, "expired")
        # Republish so volatile subscribers (web via rosbridge) converge without a
        # dedicated query; subscribers dedupe by value.
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
            res.message = "empty client_id"
            return res
        with self._state_lock:
            is_new = (
                req.client_id not in self._last_seen
            )  # a request also proves liveness
            self._last_seen[req.client_id] = time.monotonic()
            refused = req.acquire and not self._servo_powered
            locked_by = self._lock_owner if req.acquire else ""
            if req.acquire and not refused and not locked_by:
                self._active_client = req.client_id
            elif not req.acquire and self._active_client == req.client_id:
                self._active_client = ""
            res.active_client = self._active_client
        if is_new:
            self._publish_count()
        if refused:
            res.success = False
            res.message = NO_SERVO_POWER
            return res
        if locked_by:
            self.get_logger().warning(
                f"Control refused to {req.client_id}: locked by {locked_by}"
            )
            res.success = False
            res.message = f"control is locked by {locked_by}"
            return res
        self.get_logger().info(
            f"Control {'granted to' if req.acquire else 'released by'} {req.client_id}"
        )
        self._publish_active()
        res.success = True
        res.message = "ok"
        return res

    def _on_control_lock(
        self, req: ControlLock.Request, res: ControlLock.Response
    ) -> ControlLock.Response:
        """Hold control free. Atomic with `_on_control`, and fails while someone has control."""
        if not req.owner:
            res.success = False
            res.message = "empty owner"
            return res
        with self._state_lock:
            res.active_client = self._active_client
            owns_or_free = self._lock_owner in ("", req.owner)
            if req.acquire:
                if not owns_or_free:
                    res.success = False
                    res.message = f"already locked by {self._lock_owner}"
                elif self._active_client:
                    res.success = False
                    res.message = f"{self._active_client} has control"
                else:
                    self._lock_owner = req.owner
                    res.success = True
                    res.message = "locked"
            elif owns_or_free:
                self._lock_owner = ""
                res.success = True
                res.message = "released"
            else:
                res.success = False
                res.message = f"locked by {self._lock_owner}"
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
    from ..config_store import ConfigStore
    from ..main import _resolve_paths

    rclpy.init()
    bootstrap = rclpy.create_node("lucy_client_registry_bootstrap")
    robot_package = bootstrap.declare_parameter("robot_package", "thais_urdf").value
    config_dir = bootstrap.declare_parameter("config_dir", "").value
    bootstrap.destroy_node()
    store = ConfigStore(_resolve_paths(robot_package, config_dir).config_dir)
    node = ClientRegistryNode(read_active_config=store.read_active_yaml)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()

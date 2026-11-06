"""
ROS interface of the calibration wizard, and of arm identification at boot.

A separate process, so that a fault here can't take down config services or the
client registry.
"""

from __future__ import annotations

import json
import threading

from lucy_msgs.srv import CalibrationCommand
from lucy_msgs.srv import ControlLock
from lucy_msgs.srv import GetHardwareIdentity
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from std_msgs.msg import String
import yaml

from .client_registry_node import CONTROL_LOCK_SERVICE
from ..calibration.firmware_client import FirmwareBackend
from ..calibration.firmware_client import FirmwareUnreachable
from ..calibration.identity import decide
from ..calibration.identity import Identity
from ..calibration.session import IDLE
from ..calibration.session import load_profile
from ..calibration.session import Session
from ..calibration.session import SessionError
from ..calibration.sim_backend import SimBackend
from ..config_store import ConfigStore
from ..main import _resolve_paths

COMMAND_SERVICE = '/lucy/calibration/command'
STATUS_TOPIC = '/lucy/calibration/status'
IDENTITY_SERVICE = '/lucy/hardware_identity'
LOCK_OWNER = 'calibration'
LOCK_TIMEOUT_S = 2.0
# The firmware answers its socket once per 50 ms control tick, so a poll takes up to that long.
ACTIVE_PERIOD_S = 0.1
IDLE_POLLS = 10  # Polls between status refreshes when no session is running (1 Hz).
GATE_POLLS = 50  # Polls between deciding whether this arm may be driven (0.2 Hz).

_LATCHED_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


class RegistryLock:
    """The client registry's control lock, called from executor threads."""

    def __init__(self, node: Node, group) -> None:
        self._client = node.create_client(ControlLock, CONTROL_LOCK_SERVICE, callback_group=group)

    def _call(self, acquire: bool) -> bool:
        if not self._client.service_is_ready():
            return False
        done = threading.Event()
        future = self._client.call_async(ControlLock.Request(owner=LOCK_OWNER, acquire=acquire))
        future.add_done_callback(lambda _future: done.set())
        if not done.wait(LOCK_TIMEOUT_S):
            future.cancel()
            return False
        result = future.result()
        return bool(result and result.success)

    def acquire(self) -> bool:
        return self._call(True)

    def release(self) -> bool:
        return self._call(False)


class CalibrationNode(Node):
    """Serves wizard commands and publishes the session state."""

    def __init__(self) -> None:
        super().__init__('lucy_calibration')
        self.declare_parameter('robot_package', 'thais_urdf')
        self.declare_parameter('config_dir', '')
        # Fail-safe default: the simulated arm must be asked for, never fallen into.
        self.declare_parameter(
            'real', True, ParameterDescriptor(dynamic_typing=True)
        )
        robot_package = self.get_parameter('robot_package').get_parameter_value().string_value
        config_dir = self.get_parameter('config_dir').get_parameter_value().string_value
        real = self.get_parameter('real').value
        self._real = str(real).strip().lower() not in ('false', '0', 'no')

        self._store = ConfigStore(_resolve_paths(robot_package, config_dir).config_dir)
        self._profile = load_profile(self._store)
        self._mutex = threading.Lock()
        self._polls = 0
        self._session: Session | None = None

        reentrant = ReentrantCallbackGroup()
        # Requests and polling in separate groups, serialized by self._mutex instead: the
        # executor serves ready timers first, so in one group a poller that is always due
        # would keep every request waiting.
        request_group = MutuallyExclusiveCallbackGroup()
        poll_group = MutuallyExclusiveCallbackGroup()
        self._status_pub = self.create_publisher(String, STATUS_TOPIC, _LATCHED_QOS)
        self.create_service(
            GetHardwareIdentity, IDENTITY_SERVICE, self._on_identity, callback_group=request_group
        )

        if self._profile is None:
            self._status_pub.publish(String(data=json.dumps({'phase': 'unsupported'})))
            self.get_logger().info(f'{robot_package} has no calibration profile')
            return

        ids = [j['id'] for j in self._profile['joints']]
        self._backend = (
            FirmwareBackend(str(self._profile['host_node'])) if self._real else SimBackend(ids)
        )
        self._lock = RegistryLock(self, reentrant)
        self._session = Session(
            backend=self._backend,
            store=self._store,
            profile=self._profile,
            lock=self._lock,
        )
        self.create_service(
            CalibrationCommand, COMMAND_SERVICE, self._on_command, callback_group=request_group
        )
        self.create_timer(ACTIVE_PERIOD_S, self._on_timer, callback_group=poll_group)
        self._selected_sent = False
        self.get_logger().info(
            f"Calibration ready ({'real hardware' if self._real else 'simulated arm'})"
        )

    def destroy_node(self) -> bool:
        if self._session is not None:
            self._backend.close()
        return super().destroy_node()

    def _publish(self) -> None:
        self._status_pub.publish(String(data=json.dumps(self._session.status())))

    def _restore_selection(self) -> None:
        """Tell a firmware that just started which of several adapters was chosen."""
        if self._selected_sent:
            return
        selected = self._store.selected_adapter()
        try:
            if selected:
                self._backend.select(selected)
            self._selected_sent = True
        except FirmwareUnreachable:
            pass

    def _gate_control(self) -> None:
        """Keep real hardware from being driven until the arm is known to fit the active preset."""
        if not self._real or self._session.phase != IDLE:
            return
        if self._identity().control_allowed:
            self._lock.release()
        else:
            self._lock.acquire()

    def _on_timer(self) -> None:
        self._polls += 1
        gate = self._polls % GATE_POLLS == 1
        if self._session.phase == IDLE and self._polls % IDLE_POLLS and not gate:
            return
        with self._mutex:
            self._restore_selection()
            self._session.poll()
            if gate:
                self._gate_control()
            self._publish()

    def _on_command(
        self, req: CalibrationCommand.Request, res: CalibrationCommand.Response
    ) -> CalibrationCommand.Response:
        with self._mutex:
            try:
                self._session.command(req.command, req.argument)
                res.success = True
            except SessionError as e:
                res.success = False
                res.error = e.code
                res.message = str(e)
                self.get_logger().warning(f'{req.command} refused: {e}')
            status = self._session.status()
            if res.success:
                res.message = status['message']
            res.status_json = json.dumps(status)
            self._publish()
        return res

    def _identity(self) -> Identity:
        if self._profile is None:
            return Identity('unsupported', control_allowed=True)
        if not self._real:
            return Identity('simulated', control_allowed=True, config_active=True)
        try:
            status = self._backend.status()
        except FirmwareUnreachable:
            status = None
        homing = None
        if status is not None and status.get('link') == 'open':
            try:
                reply = self._backend.read_homing()
                homing = reply.get('homing') if reply.get('ok') else None
            except FirmwareUnreachable:
                pass
        return decide(
            status=status,
            store=self._store,
            homing=homing,
            joint_ids_by_actuator=self._actuator_ids(),
        )

    def _actuator_ids(self) -> dict[str, int]:
        try:
            config = yaml.safe_load(self._store.read_active_yaml())
        except (OSError, yaml.YAMLError):
            return {}
        return {
            str(a['id']): int(a['physical_pin'])
            for a in (config or {}).get('actuators') or []
            if a.get('physical_pin') is not None
        }

    def _on_identity(
        self, _req: GetHardwareIdentity.Request, res: GetHardwareIdentity.Response
    ) -> GetHardwareIdentity.Response:
        with self._mutex:
            identity = self._identity()
        res.state = identity.state
        res.adapter_serial = identity.adapter_serial
        res.matched_config = identity.matched_config
        res.config_active = identity.config_active
        res.control_allowed = identity.control_allowed
        res.message = identity.message
        return res


def main() -> None:  # pragma: no cover
    rclpy.init()
    node = CalibrationNode()
    # A request, a poll, and the control lock's response can all be in flight at once.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':  # pragma: no cover
    main()

"""The client registry's control lock, called from executor threads."""

from __future__ import annotations

from lucy_msgs.srv import ControlLock
from rclpy.node import Node

from ..ros_wait import wait_for_future
from .client_registry_node import CONTROL_LOCK_SERVICE

LOCK_OWNER = "calibration"
LOCK_TIMEOUT_S = 2.0


class RegistryLock:
    def __init__(self, node: Node, group) -> None:
        self._client = node.create_client(
            ControlLock, CONTROL_LOCK_SERVICE, callback_group=group
        )

    def _call(self, acquire: bool) -> bool:
        if not self._client.service_is_ready():
            return False
        future = self._client.call_async(
            ControlLock.Request(owner=LOCK_OWNER, acquire=acquire)
        )
        if not wait_for_future(future, LOCK_TIMEOUT_S):
            future.cancel()
            return False
        result = future.result()
        return result is not None and result.success

    def acquire(self) -> bool:
        return self._call(True)

    def release(self) -> bool:
        return self._call(False)

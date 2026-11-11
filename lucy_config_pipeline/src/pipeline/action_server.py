from __future__ import annotations

import os
import threading

from lucy_msgs.action import ConfigurePipeline
from rclpy.action import ActionServer
from rclpy.action import CancelResponse
from rclpy.action import GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_srvs.srv import Trigger

from .execute import run_execute
from .models import PipelinePaths
from ..config_store import ConfigStore


class PipelineActionServer(Node):
    _BUILD_TIMEOUT_SECONDS = int(
        os.environ.get("LUCY_PIPELINE_BUILD_TIMEOUT_SEC", "300")
    )
    _RELOAD_SERVICE = "/lucy_control/restart"
    _RELOAD_TIMEOUT_SECONDS = 15.0
    _RELOAD_RESTART_TIMEOUT_SECONDS = 90.0

    def __init__(self, *, paths: PipelinePaths, config_store: ConfigStore):
        super().__init__("lucy_config_pipeline")
        self._paths = paths
        self._store = config_store
        self._busy_lock = threading.Lock()
        self._busy = False
        # The action's _execute callback blocks while waiting for the reload
        # service response. To avoid a single-callback-group deadlock (and the
        # related corruption from re-entering the executor via
        # spin_until_future_complete), give the action and the reload client
        # independent callback groups so the response can be dispatched on a
        # different thread while _execute is still running.
        self._action_cb_group = MutuallyExclusiveCallbackGroup()
        self._reload_cb_group = ReentrantCallbackGroup()
        self._reload_client = self.create_client(
            Trigger, self._RELOAD_SERVICE, callback_group=self._reload_cb_group
        )
        self._action_server = ActionServer(
            self,
            ConfigurePipeline,
            "configure_pipeline",
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._action_cb_group,
        )

    def destroy_node(self) -> bool:
        self._action_server.destroy()
        return super().destroy_node()

    def _goal_callback(self, _goal_request: ConfigurePipeline.Goal) -> GoalResponse:
        with self._busy_lock:
            if self._busy:
                return GoalResponse.REJECT
            self._busy = True
            return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        try:
            return run_execute(
                goal_handle,
                node=self,
                store=self._store,
                paths=self._paths,
                reload_client=self._reload_client,
                reload_service=self._RELOAD_SERVICE,
                reload_timeout_seconds=self._RELOAD_TIMEOUT_SECONDS,
                reload_restart_timeout_seconds=self._RELOAD_RESTART_TIMEOUT_SECONDS,
                build_timeout_seconds=self._BUILD_TIMEOUT_SECONDS,
                log_error=self.get_logger().error,
            )
        finally:
            with self._busy_lock:
                self._busy = False

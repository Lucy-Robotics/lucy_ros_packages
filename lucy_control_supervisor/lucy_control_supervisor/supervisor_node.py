# Copyright 2026 Lucy Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

"""Manage RSP + ros2_control_node; restart on /lucy_control/restart.

Spawners are started by each robot package's control.launch.py, not here."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from dataclasses import field
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import threading
import time
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
import yaml



_WINDOWS_EXEC_SUFFIXES = ('.exe', '.bat', '.cmd')


def _control_ready_file() -> Path:
    """Return where the supervisor records that the control stack is up.

    Lets the launcher's readiness probe be a file read rather than a graph query
    charged a Pixi activation and an rclpy node.
    """
    override = os.environ.get('LUCY_CONTROL_READY_FILE')
    if override:
        return Path(override)
    uid = getattr(os, 'getuid', lambda: 0)()
    return Path(tempfile.gettempdir()) / f'lucy_control_ready.{uid}'


def node_argv(package: str, executable: str) -> list[str]:
    """Absolute argv for a node, bypassing the ``ros2 run`` wrapper.

    Signalling that wrapper leaves the node running, so a restart would stack a
    second controller_manager on the orphaned first and both drive the joints.
    """
    from ament_index_python.packages import get_package_prefix

    lib_dir = Path(get_package_prefix(package)) / 'lib' / package
    matches = sorted(
        (p for p in lib_dir.glob('*') if p.is_file() and p.stem == executable),
        # Prefer something the OS can exec over a script needing an interpreter.
        key=lambda p: 0 if not p.suffix or p.suffix.lower() in _WINDOWS_EXEC_SUFFIXES else 1,
    )
    if not matches:
        raise RuntimeError(f'{package}/{executable} not found in {lib_dir}')
    best = matches[0]
    if os.name == 'nt' and best.suffix.lower() not in _WINDOWS_EXEC_SUFFIXES:
        return [sys.executable, str(best)]
    return [str(best)]


@dataclass
class _ManagedProc:
    name: str
    popen: subprocess.Popen
    kind: str  # rsp | cm
    stopping: bool = False  # set when we ask it to stop, so the exit is expected
    # Temp files the child reads while starting. Removed when it is reaped rather
    # than on a graph condition, which another node can satisfy first.
    cleanup_paths: tuple = ()


@dataclass
class _StackConfig:
    urdf_path: Path
    base_path: Path
    controllers_yaml: Path
    use_gazebo_sim: bool
    use_mock_hardware: bool
    gazebo_only: bool
    ros2_control_file: str


class ControlSupervisorNode(Node):
    RESTART_SERVICE = '/lucy_control/restart'
    GAZEBO_RUNNING_TOPIC = '/lucy/gazebo_running'
    TERMINATE_TIMEOUT_S = 10.0
    READY_TIMEOUT_S = 60.0
    CONTROL_READY_FILE = _control_ready_file()
    CHILD_LOG_TAIL = 50
    DISCOVERY_SETTLE_S = 2.0
    CONTROLLER_MANAGER_NODE = 'controller_manager'

    def __init__(self) -> None:
        super().__init__('lucy_control_supervisor')
        self.declare_parameter('urdf_path', '')
        self.declare_parameter('base_path', '')
        self.declare_parameter('controllers_yaml', '')
        self.declare_parameter('use_gazebo_sim', False)
        self.declare_parameter('use_mock_hardware', False)
        self.declare_parameter('gazebo_only', False)
        self.declare_parameter('autostart', True)
        self.declare_parameter('ros2_control_file', 'inmoov_ros2_control.xacro')

        self._children: List[_ManagedProc] = []
        self._restart_lock = False
        self._srv = self.create_service(Trigger, self.RESTART_SERVICE, self._on_restart)
        # Latched publisher so any late subscriber (e.g. the LCP after rosbridge reconnects)
        # immediately sees whether Gazebo is part of the current launch.
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._gazebo_running_pub = self.create_publisher(
            Bool, self.GAZEBO_RUNNING_TOPIC, latched_qos
        )
        self._publish_gazebo_running()
        if self.get_parameter('autostart').value:
            ok, msg = self._restart_stack()
            if not ok:
                self.get_logger().error(msg)

    def _publish_gazebo_running(self) -> None:
        msg = Bool()
        msg.data = bool(self.get_parameter('use_gazebo_sim').value)
        self._gazebo_running_pub.publish(msg)

    def destroy_node(self) -> bool:
        self._terminate_children()
        return super().destroy_node()

    def _cfg(self) -> _StackConfig:
        return _StackConfig(
            urdf_path=Path(self.get_parameter('urdf_path').value).resolve(),
            base_path=Path(self.get_parameter('base_path').value).resolve(),
            controllers_yaml=Path(self.get_parameter('controllers_yaml').value).resolve(),
            use_gazebo_sim=bool(self.get_parameter('use_gazebo_sim').value),
            use_mock_hardware=bool(self.get_parameter('use_mock_hardware').value),
            gazebo_only=bool(self.get_parameter('gazebo_only').value),
            ros2_control_file=str(
                self.get_parameter('ros2_control_file').value or 'inmoov_ros2_control.xacro'
            ).strip(),
        )

    def _xacro_cmd(self, cfg: _StackConfig) -> list[str]:
        tail = [
            str(cfg.urdf_path),
            f'base_path:={cfg.base_path}',
            f'controller_config:={cfg.controllers_yaml}',
            f"use_gazebo_sim:={'true' if cfg.use_gazebo_sim else 'false'}",
            f"use_mock_hardware:={'true' if cfg.use_mock_hardware else 'false'}",
            f'ros2_control_file:={cfg.ros2_control_file}',
        ]
        try:
            return [*node_argv('xacro', 'xacro'), *tail]
        except (RuntimeError, KeyError):  # KeyError: package not in the index
            pass
        if shutil.which('xacro'):
            return ['xacro', *tail]
        raise RuntimeError('xacro not found')

    def _expand_robot_description(self, cfg: _StackConfig) -> str:
        cmd = self._xacro_cmd(cfg)
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=120, check=False)
        if r.returncode != 0:
            raise RuntimeError(f'xacro failed: {r.stderr or r.stdout}')
        return r.stdout

    def _clear_ready_marker(self) -> None:
        try:
            self.CONTROL_READY_FILE.unlink()
        except OSError:
            pass

    def _terminate_children(self) -> None:
        self._clear_ready_marker()
        for child in reversed(self._children):
            child.stopping = True
            if child.popen.poll() is None:
                try:
                    child.popen.terminate()
                except OSError:
                    pass
        deadline = time.monotonic() + self.TERMINATE_TIMEOUT_S
        for child in self._children:
            while child.popen.poll() is None and time.monotonic() < deadline:
                time.sleep(0.1)
        for child in self._children:
            if child.popen.poll() is None:
                self.get_logger().warning(f'{child.name} ignored terminate; killing')
                try:
                    child.popen.kill()
                except OSError:
                    pass
        for child in self._children:
            for path in child.cleanup_paths:
                try:
                    os.unlink(path)
                except OSError:
                    pass
        self._children.clear()

    def _foreign_controller_managers(self) -> List[str]:
        """controller_manager nodes running that this supervisor did not start.

        Two of them command the same joints, so the robot follows whichever
        wrote last. Usually a stack left over from an earlier run.
        """
        if any(c.kind == 'cm' and c.popen.poll() is None for c in self._children):
            return []
        # Nothing of ours is running, so give discovery a moment to report
        # anyone else's before concluding the graph is clear.
        time.sleep(self.DISCOVERY_SETTLE_S)
        return [
            f"{namespace.rstrip('/')}/{name}"
            for name, namespace in self.get_node_names_and_namespaces()
            if name == self.CONTROLLER_MANAGER_NODE
        ]

    def _wait_for(self, predicate, timeout_s: float, poll_s: float = 0.1) -> bool:
        """Poll until `predicate` holds.

        Graph queries go straight to rcl, so this is safe from a service callback.
        """
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if predicate():
                return True
            time.sleep(poll_s)
        return predicate()

    def _service_available(self, service: str) -> bool:
        return any(name == service for name, _ in self.get_service_names_and_types())

    def _start_rsp(self, cfg: _StackConfig, urdf_xml: str) -> None:
        payload = {
            'robot_state_publisher': {
                'ros__parameters': {
                    'robot_description': urdf_xml,
                    'use_sim_time': bool(cfg.use_gazebo_sim),
                }
            }
        }
        with tempfile.NamedTemporaryFile(
            mode='w', suffix='.yaml', delete=False, encoding='utf-8'
        ) as f:
            yaml.safe_dump(payload, f, sort_keys=False)
            params_file = f.name
        cmd = [
            *node_argv('robot_state_publisher', 'robot_state_publisher'),
            '--ros-args',
            '--params-file',
            params_file,
        ]
        popen = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            env=os.environ.copy(),
        )
        self._track('robot_state_publisher', popen, 'rsp', cleanup_paths=(params_file,))
        ready = self._wait_for(
            lambda: self.count_publishers('/robot_description') > 0,
            self.READY_TIMEOUT_S,
        )
        if not ready:
            self.get_logger().warning(
                'robot_state_publisher did not publish /robot_description within '
                f'{self.READY_TIMEOUT_S:.0f}s'
            )

    def _track(
        self,
        name: str,
        popen: subprocess.Popen,
        kind: str,
        cleanup_paths: tuple = (),
    ) -> None:
        """Register a child and keep draining its output.

        Unread pipes stall a chatty child once the buffer fills. Lines go to
        debug; the tail is replayed at error level only if the child dies badly.
        """
        child = _ManagedProc(name, popen, kind, cleanup_paths=cleanup_paths)
        self._children.append(child)
        tail: deque = deque(maxlen=self.CHILD_LOG_TAIL)

        def pump() -> None:
            try:
                for line in iter(popen.stdout.readline, b''):
                    text = line.decode('utf-8', 'replace').rstrip()
                    if text:
                        tail.append(text)
                        self.get_logger().debug(f'[{name}] {text}')
            except (OSError, ValueError):
                pass
            # poll rather than wait: _terminate_children polls these same handles.
            deadline = time.monotonic() + self.TERMINATE_TIMEOUT_S
            code = popen.poll()
            while code is None and time.monotonic() < deadline:
                time.sleep(0.1)
                code = popen.poll()
            if code and not child.stopping:
                self.get_logger().error(f'{name} exited with code {code}')
                for text in tail:
                    self.get_logger().error(f'[{name}] {text}')

        threading.Thread(target=pump, name=f'pump-{name}', daemon=True).start()

    def _start_cm(self, cfg: _StackConfig) -> None:
        cmd = [
            *node_argv('controller_manager', 'ros2_control_node'),
            '--ros-args',
            '--params-file',
            str(cfg.controllers_yaml),
            '-r',
            '~/robot_description:=/robot_description',
        ]
        popen = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            env=os.environ.copy(),
        )
        self._track('ros2_control_node', popen, 'cm')
        # Spawners hold a global lock while waiting for this service, so starting
        # them before it answers costs each one a full 20s lock attempt.
        if not self._wait_for(
            lambda: self._service_available('/controller_manager/list_controllers'),
            self.READY_TIMEOUT_S,
        ):
            self.get_logger().warning(
                'controller_manager did not offer list_controllers within '
                f'{self.READY_TIMEOUT_S:.0f}s; spawners may block on the lock'
            )

    def _restart_stack(self) -> tuple[bool, str]:
        if self._restart_lock:
            return False, 'restart already in progress'
        self._restart_lock = True
        try:
            cfg = self._cfg()
            # config_pipeline_node generates the ros2_control xacro, so these can
            # legitimately not exist yet; waiting replaces a guessed launch delay.
            inputs = (cfg.urdf_path, cfg.base_path, cfg.controllers_yaml)
            self._wait_for(lambda: all(p.exists() for p in inputs), self.READY_TIMEOUT_S)
            for p in inputs:
                if not p.exists():
                    return False, f'missing path: {p}'

            if not cfg.use_gazebo_sim and not cfg.gazebo_only:
                foreign = self._foreign_controller_managers()
                if foreign:
                    return False, (
                        f"controller_manager already running ({', '.join(foreign)}) "
                        'and not owned by this supervisor. Two of them drive the '
                        'same joints, so refusing to start a second. Stop the '
                        'other Lucy stack, then retry.'
                    )

            self._terminate_children()
            urdf_xml = self._expand_robot_description(cfg)

            if not cfg.gazebo_only:
                self._start_rsp(cfg, urdf_xml)
                if not cfg.use_gazebo_sim:
                    self._start_cm(cfg)

            note = ''
            if cfg.use_gazebo_sim:
                note = (
                    ' Gazebo: if URDF hardware topology changed, restart Gazebo '
                    '(gz_ros2_control loads URDF at spawn).'
                )
            return True, f'control stack restarted.{note}'
        except Exception as e:
            return False, str(e)
        finally:
            self._restart_lock = False

    def _on_restart(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        ok, msg = self._restart_stack()
        response.success = ok
        response.message = msg
        if not ok:
            self.get_logger().error(msg)
        else:
            self.get_logger().info(msg)
        return response


def main() -> None:
    rclpy.init()
    node = ControlSupervisorNode()
    try:
        rclpy.spin(node)
    finally:
        # The children go with this process, so the marker must not outlive it.
        node._terminate_children()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

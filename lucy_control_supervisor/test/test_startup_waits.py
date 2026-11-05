"""Startup waits on the ROS graph rather than on fixed sleeps."""

from pathlib import Path
import time

SRC = (Path(__file__).resolve().parents[1]
       / 'lucy_control_supervisor' / 'supervisor_node.py').read_text()


def test_no_fixed_sleep_in_startup():
    # Startup waits on the graph; a fixed sleep would be a guess at someone
    # else's readiness.
    assert 'time.sleep(1.0)' not in SRC


def test_controller_manager_is_waited_for_before_handing_off():
    """Supervisor waits for CM before spawning (spawners hold a global lock)."""
    assert '/controller_manager/list_controllers' in SRC
    start_cm = SRC.index('def _start_cm')
    next_method = SRC.index('def _restart_stack')
    assert '_service_available' in SRC[start_cm:next_method]
    assert '_spawn_controllers' in SRC[start_cm:next_method]
    assert 'self._spawn_controllers(cfg)' in SRC


def test_torque_commands_wait_for_dds_match_after_spawn():
    """Volatile ~/commands needs a subscriber before the latched Control Robot flag is sent."""
    assert 'TORQUE_MATCH_TIMEOUT_S' in SRC
    assert '_publish_torque_after_match' in SRC
    assert 'wait_for_torque_match' in SRC


def test_robot_state_publisher_is_waited_for_by_topic():
    assert "count_publishers('/robot_description')" in SRC


def test_urdf_params_file_is_unlinked_with_its_child():
    """Cleanup is tied to the child, not to a graph condition.

    robot_state_publisher reads the file while constructing. Removing it as soon
    as /robot_description has a publisher deletes it under a still-starting child
    whenever some other node is already publishing.
    """
    assert "cleanup_paths=(params_file,)" in SRC
    assert 'os.unlink(path)' in SRC


def test_waits_are_bounded():
    assert 'READY_TIMEOUT_S' in SRC


def test_discovery_settle_is_kept():
    """The absence of a foreign controller_manager cannot be waited for."""
    assert 'DISCOVERY_SETTLE_S' in SRC


class _FakeGraph:

    def __init__(self, appear_after=0.0):
        self._deadline = time.time() + appear_after

    def get_service_names_and_types(self):
        if time.time() < self._deadline:
            return [('/other/service', ['x'])]
        return [('/controller_manager/list_controllers', ['x'])]


def _wait_for(predicate, timeout_s, poll_s=0.01):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(poll_s)
    return predicate()


def test_wait_returns_as_soon_as_the_condition_holds():
    graph = _FakeGraph(appear_after=0.0)
    started = time.time()
    ok = _wait_for(
        lambda: any(n == '/controller_manager/list_controllers'
                    for n, _ in graph.get_service_names_and_types()),
        timeout_s=5.0,
    )
    assert ok and time.time() - started < 0.5


def test_wait_gives_up_at_the_deadline():
    graph = _FakeGraph(appear_after=99.0)
    started = time.time()
    assert _wait_for(
        lambda: any(n == '/controller_manager/list_controllers'
                    for n, _ in graph.get_service_names_and_types()),
        timeout_s=0.3,
    ) is False
    assert time.time() - started < 2.0


def test_bridges_stop_after_the_stack_so_torque_off_reaches_the_boards():
    from types import SimpleNamespace

    from lucy_control_supervisor.supervisor_node import _ManagedProc
    from lucy_control_supervisor.supervisor_node import ControlSupervisorNode

    order: list[str] = []

    class FakePopen:
        def __init__(self, name):
            self.name, self.done = name, False

        def poll(self):
            return 0 if self.done else None

        def terminate(self):
            order.append(self.name)
            self.done = True

    children = [
        _ManagedProc('rsp', FakePopen('rsp'), 'rsp'),
        _ManagedProc('cm', FakePopen('cm'), 'cm'),
        _ManagedProc('bridge_left', FakePopen('bridge_left'), 'bridge'),
    ]
    node = SimpleNamespace(
        _children=children,
        _clear_ready_marker=lambda: None,
        BRIDGE_DRAIN_S=0.0,
        TERMINATE_TIMEOUT_S=1.0,
        get_logger=lambda: SimpleNamespace(warning=print),
    )
    node._stop_children = lambda c: ControlSupervisorNode._stop_children(node, c)
    ControlSupervisorNode._terminate_children(node)
    assert order == ['cm', 'rsp', 'bridge_left']
    assert node._children == []

"""node_argv must resolve nodes directly.

Going through `ros2 run` makes the node a grandchild, so terminating what we
hold a handle to leaves the node running and the next start stacks a second
controller_manager on it.
"""

from pathlib import Path

from lucy_control_supervisor.supervisor_node import node_argv
import pytest

NODES = [
    ('controller_manager', 'ros2_control_node'),
    ('controller_manager', 'spawner'),
    ('robot_state_publisher', 'robot_state_publisher'),
]


@pytest.mark.parametrize('package,executable', NODES)
def test_resolves_to_an_existing_file(package, executable):
    argv = node_argv(package, executable)
    assert Path(argv[-1]).is_file()
    assert Path(argv[-1]).stem == executable


@pytest.mark.parametrize('package,executable', NODES)
def test_never_delegates_to_the_ros2_wrapper(package, executable):
    argv = node_argv(package, executable)
    assert 'run' not in argv
    assert Path(argv[0]).stem != 'ros2'


def test_unknown_executable_is_reported():
    with pytest.raises(RuntimeError, match='not found'):
        node_argv('controller_manager', 'no_such_node')

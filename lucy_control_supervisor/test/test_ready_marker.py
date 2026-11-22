"""Ready marker and the single batched spawner call."""

from pathlib import Path
import subprocess
from types import SimpleNamespace

from lucy_control_supervisor import supervisor_node as sn
from lucy_control_supervisor.supervisor_node import ControlSupervisorNode
import pytest


def _node(tmp_path: Path, children):
    marker = tmp_path / "ready"
    node = SimpleNamespace(
        _children=children,
        CONTROL_READY_FILE=marker,
        SPAWNER_TIMEOUT_S=10.0,
        SPAWNER_PER_CONTROLLER_S=5.0,
        get_logger=lambda: SimpleNamespace(info=lambda *_: None, warning=print),
        _bind_torque_controllers=lambda cfg: None,
        _publish_torque_after_match=lambda: None,
    )
    node._write_ready_marker = lambda n: ControlSupervisorNode._write_ready_marker(
        node, n
    )
    return node, marker


def _child(kind, pid):
    return SimpleNamespace(kind=kind, popen=SimpleNamespace(pid=pid))


def test_marker_holds_cm_pid_and_count(tmp_path):
    node, marker = _node(tmp_path, [_child("rsp", 1), _child("cm", 4242)])
    node._write_ready_marker(7)
    assert marker.read_text() == "4242 7\n"


def test_marker_is_a_noop_without_cm(tmp_path):
    node, marker = _node(tmp_path, [_child("rsp", 1)])
    node._write_ready_marker(7)
    assert not marker.exists()


def _spawn(node, monkeypatch, names, returncode):
    calls = []

    def run(cmd, **kw):
        calls.append((cmd, kw))
        return subprocess.CompletedProcess(cmd, returncode, "", "boom")

    monkeypatch.setattr(sn.subprocess, "run", run)
    monkeypatch.setattr(sn, "node_argv", lambda p, e: [e])
    monkeypatch.setattr(sn, "controllers_to_spawn", lambda *a, **k: names)
    cfg = SimpleNamespace(controllers_yaml="x", use_gazebo_sim=False)
    ControlSupervisorNode._spawn_controllers(node, cfg)
    return calls


def test_one_spawner_for_all_controllers_then_marker(tmp_path, monkeypatch):
    node, marker = _node(tmp_path, [_child("cm", 99)])
    names = ["joint_state_broadcaster", "a", "b"]
    calls = _spawn(node, monkeypatch, names, 0)
    assert len(calls) == 1
    cmd, kw = calls[0]
    assert cmd[1:4] == names
    assert kw["timeout"] == 20.0
    assert marker.read_text() == "99 3\n"


def test_failed_spawner_raises_and_writes_no_marker(tmp_path, monkeypatch):
    node, marker = _node(tmp_path, [_child("cm", 99)])
    with pytest.raises(RuntimeError, match="boom"):
        _spawn(node, monkeypatch, ["x", "y"], 1)
    assert not marker.exists()

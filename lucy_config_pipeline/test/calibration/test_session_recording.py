"""Recording a joint's range: per-joint sweeps, wrap handling and homing checks."""

from __future__ import annotations

from session_support import calibrate_all
from session_support import code_of
from session_support import run_sweep
from session_support import sweep
from src.calibration.session import HALF_TURN_TICK
from src.calibration.session import JOINTS
from src.calibration.session import MID_CHECK_POLLS
from src.calibration.session import REVIEW
from src.calibration.sim_backend import SWEEP_PERIOD_S


def test_each_joint_records_on_its_own_and_accept_moves_on(session, arm):
    session.command("start")
    session.command("capture_mid")
    assert session.status()["recording"] == "Rotation"

    order = (("Rotation", "Pitch"), ("Pitch", "Wrist_Roll"), ("Wrist_Roll", "Jaw"))
    for joint, following in order:
        assert session.status()["recording"] == joint
        run_sweep(session, arm)
        session.command("accept_joint")
        assert session.phase == JOINTS
        assert session.status()["recording"] == following
        assert joint in session.status()["accepted"]

    run_sweep(session, arm)
    session.command("accept_joint")
    assert session.phase == REVIEW
    assert session.status()["recording"] is None


def test_redo_from_review_returns_to_review(session, arm):
    calibrate_all(session, arm)
    sweep(session, arm, "Pitch")
    session.command("accept_joint")
    assert session.phase == REVIEW


def test_homing_check_ignores_the_joint_already_being_recorded(session, arm):
    session.command("start")
    session.command("capture_mid")
    run_sweep(session, arm)  # Rotation sweeps to its stops while the check runs.
    assert not any("moved during homing" in w for w in session.status()["warnings"])


def test_positions_are_homed_after_capture(session, arm):
    session.command("start")
    session.command("capture_mid")
    assert all(j["ticks"] == HALF_TURN_TICK for j in session.status()["joints"])


def test_silent_servo_fails_health(session, arm):
    real = arm.status

    def stale():
        status = real()
        status["servos"][2]["age_ms"] = 5000
        return status

    arm.status = stale
    session.poll()
    assert not session.healthy()
    assert next(c for c in session.health() if c["id"] == "servos")["ok"] is False


def test_too_small_a_sweep_needs_an_override(session, arm):
    session.command("start")
    session.command("capture_mid")
    session.command("begin_joint", "Rotation")
    arm.clock.now += SWEEP_PERIOD_S / 100
    session.poll()
    assert code_of(session, "accept_joint") == "span_too_small"

    session.command("accept_joint", "override")
    assert "Rotation" in session.status()["accepted"]


def test_range_not_containing_mid_is_refused_even_with_override(session, arm):
    session.command("start")
    session.command("capture_mid")
    real = arm.status

    def far_side():
        status = real()
        status["servos"][0]["ticks"] = 3000
        return status

    arm.status = far_side
    session.command("begin_joint", "Rotation")
    session.poll()
    assert code_of(session, "accept_joint", "override") == "mid_outside_range"


def test_wrap_during_a_sweep_restarts_the_joint(session, arm):
    session.command("start")
    session.command("capture_mid")
    session.command("begin_joint", "Rotation")
    real = arm.status
    ticks = iter([100, 4000])

    def jumping():
        status = real()
        status["servos"][0]["ticks"] = next(ticks, 4000)
        return status

    arm.status = jumping
    session.poll()
    session.poll()

    state = session.status()
    assert state["range"] == [4000, 4000]
    assert any("wrap" in w for w in state["warnings"])


def test_wrist_roll_may_wrap(session, arm):
    session.command("start")
    session.command("capture_mid")
    session.command("begin_joint", "Wrist_Roll")
    real = arm.status
    ticks = iter([3900, 4090, 10, 400, 1200])

    def turning():
        status = real()
        status["servos"][2]["ticks"] = next(ticks, 1200)
        return status

    arm.status = turning
    for _ in range(5):
        session.poll()
    assert session.status()["warnings"] == []
    session.command("accept_joint", "override")
    assert session.status()["accepted"]["Wrist_Roll"] == [0, 4095]


def test_arm_moved_during_homing_warns_only_once_positions_are_current(session, arm):
    session.command("start")
    session.command("capture_mid")
    assert session.status()["warnings"] == []

    real = arm.status

    def sagged():
        status = real()
        status["servos"][1]["ticks"] = HALF_TURN_TICK + 900
        return status

    arm.status = sagged
    for _ in range(MID_CHECK_POLLS):
        session.poll()
    assert any("Pitch" in w for w in session.status()["warnings"])

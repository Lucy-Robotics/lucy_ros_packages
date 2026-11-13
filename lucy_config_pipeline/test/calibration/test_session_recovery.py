"""Cancelling, restarting mid-session and recovering a lost control lock."""

from __future__ import annotations

from conftest import FakeLock
from session_support import apply_all
from session_support import code_of
from session_support import reborn_session
from session_support import sweep
from src.calibration.session import IDLE
from src.calibration.session import JOINTS
from src.calibration.session import RECOVERY
from src.calibration.sim_backend import SIM_SERIAL


def test_cancel_before_homing_touches_nothing(session, arm, lock):
    session.command("start")
    session.command("cancel")
    assert session.phase == IDLE
    assert arm.restores == []
    assert lock.held is False


def test_cancel_after_homing_restores_the_old_values(session, arm, store, lock):
    before = arm.read_homing()["homing"]
    session.command("start")
    session.command("capture_mid")
    assert arm.read_homing()["homing"] != before

    session.command("cancel")

    assert arm.read_homing()["homing"] == before
    assert session.phase == IDLE
    assert store.read_journal() == {}
    assert lock.held is False


def test_failed_restore_keeps_the_journal_and_can_be_retried(session, arm, store):
    session.command("start")
    session.command("capture_mid")
    arm.dead_on = "restore_homing"

    assert code_of(session, "cancel") == "firmware_unreachable"
    assert session.phase == JOINTS
    assert store.read_journal()["old"]

    arm.dead_on = ""
    session.command("cancel")
    assert session.phase == IDLE


def test_cancel_after_activation_keeps_the_new_calibration(session, arm, store):
    apply_all(session, arm)
    store.activate(session.status()["preset"])

    session.command("cancel")

    assert arm.restores == [], "the active preset was built for these offsets"
    assert session.phase == IDLE


def test_cancel_after_apply_without_activation_restores(session, arm):
    apply_all(session, arm)
    session.command("cancel")
    assert len(arm.restores) == 1


def test_restart_during_a_session_offers_recovery(session, arm, store, profile, lock):
    before = arm.read_homing()["homing"]
    session.command("start")
    session.command("capture_mid")
    sweep(session, arm, "Rotation")
    session.command("accept_joint")

    reborn = reborn_session(arm, store, profile, lock)
    assert reborn.phase == RECOVERY

    reborn.command("resume")
    assert reborn.phase == JOINTS
    assert "Rotation" in reborn.status()["accepted"]
    assert reborn.status()["recording"] == "Pitch"

    reborn.command("cancel")
    assert arm.read_homing()["homing"] == before


def test_restore_from_recovery(session, arm, store, profile, lock):
    before = arm.read_homing()["homing"]
    session.command("start")
    session.command("capture_mid")

    reborn = reborn_session(arm, store, profile, lock)
    reborn.command("restore")

    assert arm.read_homing()["homing"] == before
    assert reborn.phase == IDLE


def test_discard_from_recovery_touches_nothing(session, arm, store, profile, lock):
    session.command("start")
    session.command("capture_mid")

    reborn = reborn_session(arm, store, profile, lock)
    reborn.command("discard")

    assert arm.restores == []
    assert reborn.phase == IDLE
    assert store.read_journal() == {}


def test_discard_works_even_when_restore_would_be_refused(
    session, arm, store, profile, lock
):
    session.command("start")
    session.command("capture_mid")

    reborn = reborn_session(arm, store, profile, lock)
    reborn._serial = "a different adapter"
    assert code_of(reborn, "restore") == "serial_mismatch"
    attempts = len(arm.restores)

    reborn.command("discard")

    assert len(arm.restores) == attempts, "discard must not call the firmware"
    assert reborn.phase == IDLE


def test_resume_is_refused_if_homing_never_finished(session, arm, store, profile, lock):
    session.command("start")
    arm.dead_on = "set_half_turn"
    code_of(session, "capture_mid")
    arm.dead_on = ""

    reborn = reborn_session(arm, store, profile, lock)
    assert code_of(reborn, "resume") == "cannot_resume"


def test_resume_is_refused_if_the_servos_changed_meanwhile(
    session, arm, store, profile, lock
):
    session.command("start")
    session.command("capture_mid")
    arm.restore_homing(
        SIM_SERIAL,
        [
            {"id": r["id"], "homing_offset": 7, "min_limit": 0, "max_limit": 4095}
            for r in arm.read_homing()["homing"]
        ],
    )

    reborn = reborn_session(arm, store, profile, lock)
    assert code_of(reborn, "resume") == "homing_changed"


def test_recovery_reacquires_the_control_lock(session, arm, store, profile):
    session.command("start")
    session.command("capture_mid")

    fresh_lock = FakeLock()
    reborn = reborn_session(arm, store, profile, fresh_lock)
    reborn.poll()
    assert fresh_lock.held is True


def test_lost_lock_is_taken_back_and_shows_in_health(session, lock):
    session.command("start")
    lock.refuse = True
    session._lock_held = False
    session.poll()
    assert next(c for c in session.health() if c["id"] == "control_lock")["ok"] is False
    lock.refuse = False
    session.poll()
    assert lock.held is True

"""The wizard's state machine: starting, homing, verifying and finishing a session."""

from __future__ import annotations

from session_support import apply_all
from session_support import code_of
from session_support import patch_status
from src.calibration.session import IDLE
from src.calibration.session import MID_POSE
from src.calibration.session import RECOVERY
from src.calibration.session import REVIEW
from src.calibration.session import VERIFY_DRIVE


def test_homing_journals_old_values_before_touching_the_eeprom(session, arm, store):
    session.command("start")
    seen = {}
    real = arm.set_half_turn

    def spy(serial):
        seen["journal"] = store.read_journal()
        return real(serial)

    arm.set_half_turn = spy
    session.command("capture_mid")

    assert seen["journal"]["old"][0]["homing_offset"] == 0
    assert store.read_journal()["new"], "the result is journaled too"


def test_start_is_refused_while_someone_has_control(session, lock):
    lock.refuse = True
    assert code_of(session, "start") == "control_active"
    assert session.phase == IDLE


def test_capture_is_refused_with_a_failing_health_check(session, arm):
    session.command("start")

    def torque_on(status):
        status["servos"][1]["torque"] = True

    patch_status(arm, torque_on)
    assert code_of(session, "capture_mid") == "unhealthy"
    assert arm.halves == 0
    assert session.phase == MID_POSE


def test_capture_is_refused_with_an_unusable_serial(session, arm):
    session.command("start")

    def no_serial(status):
        status["open_serial"] = None

    patch_status(arm, no_serial)
    assert code_of(session, "capture_mid") == "unhealthy"
    assert arm.halves == 0


def test_failed_read_before_homing_changes_nothing(session, arm, store):
    session.command("start")
    arm.dead_on = "read_homing"
    assert code_of(session, "capture_mid") == "firmware_unreachable"
    assert arm.halves == 0
    assert session.phase == MID_POSE
    assert store.read_journal() == {}


def test_homing_that_restored_itself_leaves_no_journal(session, arm, store):
    session.command("start")
    arm.fail_homing_with = {
        "ok": False,
        "error": "offset_out_of_range",
        "servo": 2,
        "restored": True,
    }
    assert code_of(session, "capture_mid") == "offset_out_of_range"
    assert session.phase == MID_POSE
    assert store.read_journal() == {}


def test_homing_that_could_not_restore_keeps_the_old_values_for_recovery(
    session, arm, store
):
    session.command("start")
    arm.fail_homing_with = {
        "ok": False,
        "error": "readback_mismatch",
        "restored": False,
    }
    assert code_of(session, "capture_mid") == "readback_mismatch"
    assert session.phase == RECOVERY
    assert store.read_journal()["old"]
    assert "could not be restored" in session.status()["message"]


def test_losing_the_firmware_during_homing_means_recovery(session, arm, store):
    session.command("start")
    arm.dead_on = "set_half_turn"
    assert code_of(session, "capture_mid") == "firmware_unreachable"
    assert session.phase == RECOVERY
    assert store.read_journal()["old"]


def test_verify_drive_needs_the_preset_active_and_frees_control(
    session, arm, store, lock
):
    apply_all(session, arm)
    assert code_of(session, "verify_drive") == "not_activated"
    assert lock.held is True

    store.activate(session.status()["preset"])
    session.command("verify_drive")

    assert session.phase == VERIFY_DRIVE
    assert lock.held is False
    session.poll()
    assert lock.held is False, "polling must not take the lock back from the driver"


def test_back_to_review_needs_control_released(session, arm, store, lock):
    apply_all(session, arm)
    store.activate(session.status()["preset"])
    session.command("verify_drive")

    lock.refuse = True
    assert code_of(session, "back_to_review") == "control_active"
    lock.refuse = False
    session.command("back_to_review")
    assert session.phase == REVIEW


def test_finish_ends_the_session(session, arm, store, lock):
    apply_all(session, arm)
    assert code_of(session, "finish") == "not_activated"
    store.activate(session.status()["preset"])
    session.command("finish")
    assert session.phase == IDLE
    assert store.read_journal() == {}
    assert lock.held is False


def test_commands_out_of_order_are_refused(session):
    assert code_of(session, "capture_mid") == "wrong_phase"
    assert code_of(session, "apply") == "wrong_phase"
    assert code_of(session, "restore") == "wrong_phase"
    assert code_of(session, "nonsense") == "unknown_command"

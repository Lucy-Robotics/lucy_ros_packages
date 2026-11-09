"""The wizard's state machine, against a simulated arm and a fake one that misbehaves."""

from __future__ import annotations

import json

import pytest
from src.calibration.firmware_client import FirmwareUnreachable
from src.calibration.session import APPLIED
from src.calibration.session import HALF_TURN_TICK
from src.calibration.session import IDLE
from src.calibration.session import JOINTS
from src.calibration.session import MID_CHECK_POLLS
from src.calibration.session import MID_POSE
from src.calibration.session import RECOVERY
from src.calibration.session import REVIEW
from src.calibration.session import Session
from src.calibration.session import SessionError
from src.calibration.session import VERIFY_DRIVE
from src.calibration.sim_backend import SIM_SERIAL
from src.calibration.sim_backend import SimBackend
from src.calibration.sim_backend import SWEEP_PERIOD_S
import yaml


class Clock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now


class Arm(SimBackend):
    """Simulated arm with a hand on the clock, recording what reached the EEPROM."""

    def __init__(self, ids: list[int]) -> None:
        self.clock = Clock()
        super().__init__(ids, clock=self.clock)
        self.halves = 0
        self.restores: list[list[dict]] = []
        self.fail_homing_with: dict | None = None
        self.dead_on: str = ""

    def _maybe_die(self, call: str) -> None:
        if self.dead_on == call:
            raise FirmwareUnreachable(call)

    def read_homing(self) -> dict:
        self._maybe_die("read_homing")
        return super().read_homing()

    def set_half_turn(self, serial: str) -> dict:
        self._maybe_die("set_half_turn")
        self.halves += 1
        if self.fail_homing_with is not None:
            return self.fail_homing_with
        return super().set_half_turn(serial)

    def restore_homing(self, serial: str, records: list[dict]) -> dict:
        self._maybe_die("restore_homing")
        self.restores.append(records)
        return super().restore_homing(serial, records)


@pytest.fixture
def arm(profile) -> Arm:
    return Arm([j["id"] for j in profile["joints"]])


@pytest.fixture
def session(arm, store, profile, lock) -> Session:
    s = Session(
        backend=arm, store=store, profile=profile, lock=lock, clock=lambda: "NOW"
    )
    s.poll()
    return s


def sweep(session: Session, arm: Arm, joint: str) -> None:
    session.command("begin_joint", joint)
    for step in range(41):
        arm.clock.now += SWEEP_PERIOD_S / 40
        session.poll()
    assert step == 40


def calibrate_all(session: Session, arm: Arm) -> None:
    session.command("start")
    session.command("capture_mid")
    for joint in ("Rotation", "Pitch", "Wrist_Roll", "Jaw"):
        sweep(session, arm, joint)
        session.command("accept_joint")


def code_of(session: Session, command: str, argument: str = "") -> str:
    with pytest.raises(SessionError) as e:
        session.command(command, argument)
    return e.value.code


def test_full_session_builds_a_preset_that_lerobot_tools_can_read(session, arm, store):
    calibrate_all(session, arm)
    assert session.phase == REVIEW

    session.command("apply")

    assert session.phase == APPLIED
    name = f"so101_{SIM_SERIAL}"
    preset = yaml.safe_load(store.read_named_yaml(name))
    assert (
        preset["boards"]["rp2040_so_arm"]["serial_id"] == ""
    ), "must not trigger a Pico flash"
    block = preset["calibration"]
    assert block["adapter_serial"] == SIM_SERIAL
    assert block["calibrated_at"] == "NOW"
    assert set(block["homing_offsets"]) == {"rotation", "pitch", "wrist_roll", "jaw"}

    lerobot = json.loads(store.lerobot_json_path(name).read_text())
    assert set(lerobot) == {"shoulder_pan", "shoulder_lift", "wrist_roll", "gripper"}
    assert set(lerobot["gripper"]) == {
        "id",
        "drive_mode",
        "homing_offset",
        "range_min",
        "range_max",
    }
    assert lerobot["wrist_roll"]["range_min"] == 0
    assert lerobot["wrist_roll"]["range_max"] == 4095
    # Ranges are in the homed frame, around the mid pose.
    assert (
        lerobot["gripper"]["range_min"]
        < HALF_TURN_TICK
        < lerobot["gripper"]["range_max"]
    )
    assert block["homing_offsets"]["jaw"] == lerobot["gripper"]["homing_offset"]


def test_limits_are_inset_from_the_stops_except_on_the_full_turn_joint(
    session, arm, store
):
    calibrate_all(session, arm)
    session.command("apply")
    name = f"so101_{SIM_SERIAL}"
    lerobot = json.loads(store.lerobot_json_path(name).read_text())
    actuators = {
        a["id"]: a for a in yaml.safe_load(store.read_named_yaml(name))["actuators"]
    }

    measured = lerobot["gripper"]["range_min"] * 2 * 3.14159265 / 4096
    assert actuators["jaw"]["limit_min"] > measured
    assert actuators["wrist_roll"]["limit_min"] == 0


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


def run_sweep(session: Session, arm: Arm) -> None:
    for _ in range(41):
        arm.clock.now += SWEEP_PERIOD_S / 40
        session.poll()


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


def test_start_is_refused_while_someone_has_control(session, lock):
    lock.refuse = True
    assert code_of(session, "start") == "control_active"
    assert session.phase == IDLE


def test_capture_is_refused_with_a_failing_health_check(session, arm):
    session.command("start")
    real = arm.status

    def torque_on():
        status = real()
        status["servos"][1]["torque"] = True
        return status

    arm.status = torque_on
    assert code_of(session, "capture_mid") == "unhealthy"
    assert arm.halves == 0
    assert session.phase == MID_POSE


def test_capture_is_refused_with_an_unusable_serial(session, arm):
    session.command("start")
    real = arm.status

    def no_serial():
        status = real()
        status["open_serial"] = None
        return status

    arm.status = no_serial
    assert code_of(session, "capture_mid") == "unhealthy"
    assert arm.halves == 0


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


def test_cancel_before_homing_touches_nothing(session, arm, lock, store):
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
    calibrate_all(session, arm)
    session.command("apply")
    store.activate(session.status()["preset"])

    session.command("cancel")

    assert arm.restores == [], "the active preset was built for these offsets"
    assert session.phase == IDLE


def test_cancel_after_apply_without_activation_restores(session, arm, store):
    calibrate_all(session, arm)
    session.command("apply")
    session.command("cancel")
    assert len(arm.restores) == 1


def test_restart_during_a_session_offers_recovery(session, arm, store, profile, lock):
    before = arm.read_homing()["homing"]
    session.command("start")
    session.command("capture_mid")
    sweep(session, arm, "Rotation")
    session.command("accept_joint")

    reborn = Session(backend=arm, store=store, profile=profile, lock=lock)
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

    reborn = Session(backend=arm, store=store, profile=profile, lock=lock)
    reborn.command("restore")

    assert arm.read_homing()["homing"] == before
    assert reborn.phase == IDLE


def test_resume_is_refused_if_homing_never_finished(session, arm, store, profile, lock):
    session.command("start")
    arm.dead_on = "set_half_turn"
    code_of(session, "capture_mid")
    arm.dead_on = ""

    reborn = Session(backend=arm, store=store, profile=profile, lock=lock)
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

    reborn = Session(backend=arm, store=store, profile=profile, lock=lock)
    assert code_of(reborn, "resume") == "homing_changed"


def test_recovery_reacquires_the_control_lock(session, arm, store, profile):
    session.command("start")
    session.command("capture_mid")

    fresh_lock = type(session._lock)()
    reborn = Session(backend=arm, store=store, profile=profile, lock=fresh_lock)
    reborn.poll()
    assert fresh_lock.held is True


def test_lost_lock_is_taken_back_and_shows_in_health(session, arm, lock):
    session.command("start")
    lock.refuse = True
    session._lock_held = False
    session.poll()
    assert next(c for c in session.health() if c["id"] == "control_lock")["ok"] is False
    lock.refuse = False
    session.poll()
    assert lock.held is True


def test_verify_drive_needs_the_preset_active_and_frees_control(
    session, arm, store, lock
):
    calibrate_all(session, arm)
    session.command("apply")
    assert code_of(session, "verify_drive") == "not_activated"
    assert lock.held is True

    store.activate(session.status()["preset"])
    session.command("verify_drive")

    assert session.phase == VERIFY_DRIVE
    assert lock.held is False
    session.poll()
    assert lock.held is False, "polling must not take the lock back from the driver"


def test_back_to_review_needs_control_released(session, arm, store, lock):
    calibrate_all(session, arm)
    session.command("apply")
    store.activate(session.status()["preset"])
    session.command("verify_drive")

    lock.refuse = True
    assert code_of(session, "back_to_review") == "control_active"
    lock.refuse = False
    session.command("back_to_review")
    assert session.phase == REVIEW


def test_finish_ends_the_session(session, arm, store, lock):
    calibrate_all(session, arm)
    session.command("apply")
    assert code_of(session, "finish") == "not_activated"
    store.activate(session.status()["preset"])
    session.command("finish")
    assert session.phase == IDLE
    assert store.read_journal() == {}
    assert lock.held is False


def test_activating_the_preset_binds_the_adapter(session, arm, store):
    calibrate_all(session, arm)
    session.command("apply")
    preset = session.status()["preset"]
    assert store.bound_config(SIM_SERIAL) == ""
    store.activate(preset)
    assert store.bound_config(SIM_SERIAL) == preset


@pytest.mark.parametrize("name", ["default", "../evil", "has space", "a/b"])
def test_bad_preset_names_are_refused(session, arm, name):
    calibrate_all(session, arm)
    assert code_of(session, "apply", name) == "bad_preset_name"


def test_commands_out_of_order_are_refused(session):
    assert code_of(session, "capture_mid") == "wrong_phase"
    assert code_of(session, "apply") == "wrong_phase"
    assert code_of(session, "restore") == "wrong_phase"
    assert code_of(session, "nonsense") == "unknown_command"


def test_select_adapter_is_remembered(session, store):
    session.command("select_adapter", "ABC123")
    assert store.selected_adapter() == "ABC123"
    assert code_of(session, "select_adapter", 'bad"serial') == "bad_serial"


def test_unreachable_firmware_is_reported_not_fatal(session, arm):
    def dead():
        raise FirmwareUnreachable("gone")

    arm.status = dead
    session.poll()
    assert next(c for c in session.health() if c["id"] == "firmware")["ok"] is False


def test_adopting_binds_the_active_preset_to_this_adapter(session, store):
    store.write_named_yaml("mine", store.read_named_yaml("default"))
    store.activate("mine")
    session.command("adopt")
    assert store.bound_config(SIM_SERIAL) == "mine"


def test_the_stock_preset_cannot_be_adopted(session, store):
    assert code_of(session, "adopt") == "not_calibrated"
    assert store.bound_config(SIM_SERIAL) == ""

"""
The calibration wizard's state machine, free of ROS so it can be tested on its own.

Every EEPROM change is journaled first, so a crash can't lose the only copy of the
previous calibration.
"""

from __future__ import annotations

from datetime import datetime
from datetime import timezone
import json
import math
import re

from lucy_config_generator.calibration import apply_to_text
from lucy_config_generator.calibration import CalibrationError
from lucy_config_generator.calibration import homing_offsets_by_actuator
from lucy_config_generator.calibration import set_calibration_block
from lucy_config_generator.calibration import urdf_limits_from_config
from lucy_config_generator.calibration import validate_calibration
from lucy_config_generator.schema import validate_hardware_yaml
import yaml

from .firmware_client import FirmwareUnreachable
from .identity import FRESH_READ_MS
from .identity import servos_fresh
from .identity import valid_serial
from ..config_store import atomic_write_text
from ..config_store import ConfigStore

IDLE = "idle"
RECOVERY = "recovery"  # Found an unfinished session at startup.
MID_POSE = "mid_pose"  # Lock held; the EEPROM is untouched.
JOINTS = "joints"  # Homed: the arm's previous calibration only survives in the journal.
REVIEW = "review"  # Every joint accepted.
APPLIED = "applied"  # Preset written. The panel activates it.
VERIFY_DRIVE = "verify_drive"  # Control lock released so the panel can drive the arm.

MAX_TICK = 4095
HALF_TURN_TICK = 2047
# Between two samples, a jump past this can only be the encoder wrapping round.
WRAP_JUMP_TICKS = 2048
# Keeps joints off the stops just recorded (about 1.5 degrees).
MARGIN_RAD = math.radians(1.5)
# A joint this far from mid right after homing moved, or homing did not take.
MID_TOLERANCE_TICKS = 300
TEMPERATURE_WARN_C = 60
LOAD_WARN = 0.5
TEMPLATE_PRESET = "default"
# Polls (20 Hz) before trusting positions after homing: firmware reports its last tick's.
MID_CHECK_POLLS = 5
# Polls between re-asserting the control lock, in case the registry restarted.
LOCK_REFRESH_POLLS = 20
PRESET_NAME = re.compile(r"[A-Za-z0-9_-]+")


class SessionError(RuntimeError):
    """A command that can't run now. ``code`` is stable for the panel to act on."""

    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code


def load_profile(store: ConfigStore) -> dict | None:
    """Return this robot's calibration profile, or None if it doesn't support the wizard."""
    if not store.calibration_profile_yaml.is_file():
        return None
    return yaml.safe_load(store.calibration_profile_yaml.read_text(encoding="utf-8"))


def _utc_now() -> str:
    return (
        datetime.now(timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z")
    )


class Session:
    """One arm being calibrated. Callers serialize `command` and `poll`."""

    def __init__(
        self, *, backend, store: ConfigStore, profile: dict, lock, clock=_utc_now
    ) -> None:
        self._backend = backend
        self._store = store
        self._joints: list[dict] = profile["joints"]
        self._prefix = str(profile.get("preset_prefix") or "calibrated")
        self._controller_topic = str(profile.get("controller_topic") or "")
        self._lock = lock
        self._clock = clock

        self._firmware: dict | None = None
        self._lock_held = False
        self._message = ""
        self._warnings: list[str] = []
        self._phase = IDLE
        self._serial = ""
        self._old: list[dict] = []
        self._new: list[dict] = []
        self._accepted: dict[str, list[int]] = {}
        self._preset = ""
        self._recording: str | None = None
        self._range: list[int] | None = None
        self._last_tick: int | None = None
        self._mid_check_in = 0
        self._polls = 0

        journal = store.read_journal()
        if journal:
            self._phase = RECOVERY
            self._serial = str(journal.get("serial") or "")
            self._old = list(journal.get("old") or [])
            self._new = list(journal.get("new") or [])
            self._accepted = dict(journal.get("accepted") or {})
            self._preset = str(journal.get("preset") or "")
            self._message = "A calibration was interrupted: resume or restore it."

    @property
    def phase(self) -> str:
        return self._phase

    # --- status ---------------------------------------------------------------------------

    def poll(self) -> None:
        """Refresh from the firmware, and record the joint being swept."""
        try:
            self._firmware = self._backend.status()
        except FirmwareUnreachable:
            self._firmware = None
        self._polls += 1
        if self._phase not in (IDLE, VERIFY_DRIVE) and (
            not self._lock_held or self._polls % LOCK_REFRESH_POLLS == 0
        ):
            self._lock_held = self._lock.acquire()
        self._after_homing()
        self._record()

    def _ticks(self, joint_id: int) -> int | None:
        for servo in (self._firmware or {}).get("servos") or []:
            if servo.get("id") == joint_id and servo.get("age_ms") is not None:
                if servo["age_ms"] < FRESH_READ_MS:
                    return int(servo["ticks"])
        return None

    def _after_homing(self) -> None:
        if self._mid_check_in == 0:
            return
        self._mid_check_in -= 1
        if self._mid_check_in > 0:
            return
        # The joint being recorded may already be on its way to a stop.
        far = [
            j["name"]
            for j in self._joints
            if j["name"] != self._recording
            and (t := self._ticks(j["id"])) is not None
            and abs(t - HALF_TURN_TICK) > MID_TOLERANCE_TICKS
        ]
        if far:
            self._warnings.append(
                f"{', '.join(far)} moved during homing. If the arm was not held at the mid "
                "pose, cancel and start again."
            )

    def _record(self) -> None:
        if self._recording is None:
            return
        joint = self._joint(self._recording)
        tick = self._ticks(joint["id"])
        if tick is None:
            return
        jumped = (
            self._last_tick is not None
            and abs(tick - self._last_tick) > WRAP_JUMP_TICKS
        )
        if jumped:
            self._range = None
            if not joint.get("full_turn"):
                self._warnings.append(
                    f"{joint['name']} passed the encoder's wrap point, more than half a turn "
                    "from the mid pose. Return it to the mid pose and sweep again."
                )
        self._last_tick = tick
        if self._range is None:
            self._range = [tick, tick]
        self._range = [min(self._range[0], tick), max(self._range[1], tick)]

    def health(self) -> list[dict]:
        """Every check, so the panel can show what to fix."""
        fw = self._firmware
        servos = (fw or {}).get("servos") or []
        checks = [
            ("firmware", True, fw is not None, "Host firmware is running"),
            (
                "adapter",
                True,
                bool(fw) and fw.get("link") == "open",
                "USB adapter is open",
            ),
            (
                "serial",
                True,
                bool(fw) and valid_serial(fw.get("open_serial")),
                "USB adapter has a usable serial number",
            ),
            ("servos", True, bool(fw) and servos_fresh(fw), "All servos are answering"),
            (
                "torque",
                True,
                bool(servos) and not any(s.get("torque") for s in servos),
                "Torque is off",
            ),
            ("control_lock", True, self._lock_held, "Control Robot is off and locked"),
            (
                "temperature",
                False,
                all(s.get("temperature", 0) < TEMPERATURE_WARN_C for s in servos),
                "Servos are not overheating",
            ),
            (
                "load",
                False,
                all(abs(s.get("load", 0)) < LOAD_WARN for s in servos),
                "No servo is under unusual load",
            ),
        ]
        return [
            {"id": id_, "required": required, "ok": bool(ok), "label": label}
            for id_, required, ok, label in checks
        ]

    def healthy(self) -> bool:
        return all(c["ok"] for c in self.health() if c["required"])

    def status(self) -> dict:
        return {
            "phase": self._phase,
            "message": self._message,
            "warnings": list(self._warnings),
            "health": self.health(),
            "link": (self._firmware or {}).get("link"),
            "adapters": (self._firmware or {}).get("adapters") or [],
            "serial": self._serial,
            "preset": self._preset,
            "controller_topic": self._controller_topic,
            "recording": self._recording,
            "range": self._range,
            "accepted": self._accepted,
            "joints": [
                {
                    "name": j["name"],
                    "min_span": j["min_span"],
                    "full_turn": bool(j.get("full_turn")),
                    "ticks": self._ticks(j["id"]),
                }
                for j in self._joints
            ],
        }

    # --- commands -------------------------------------------------------------------------

    def command(self, name: str, argument: str = "") -> None:
        """Run one wizard command, raising SessionError if it can't run now."""
        handler = {
            "start": self._start,
            "capture_mid": self._capture_mid,
            "begin_joint": self._begin_joint,
            "accept_joint": self._accept_joint,
            "apply": self._apply,
            "cancel": self._cancel,
            "resume": self._resume,
            "restore": self._restore,
            "verify_drive": self._verify_drive,
            "back_to_review": self._back_to_review,
            "finish": self._finish,
            "select_adapter": self._select_adapter,
            "adopt": self._adopt,
        }.get(name)
        if handler is None:
            raise SessionError("unknown_command", f"Unknown command: {name}")
        self._warnings = []
        self._message = ""
        handler(argument)
        self.poll()

    def _need_phase(self, *allowed: str) -> None:
        if self._phase not in allowed:
            raise SessionError("wrong_phase", f"Not possible during {self._phase}.")

    def _joint(self, name: str) -> dict:
        for joint in self._joints:
            if joint["name"] == name:
                return joint
        raise SessionError("unknown_joint", f"Unknown joint: {name}")

    def _call(self, reply: dict) -> dict:
        if not reply.get("ok"):
            raise SessionError(
                str(reply.get("error") or "firmware_error"), self._explain(reply)
            )
        return reply

    @staticmethod
    def _call_firmware(
        fn, *args, message: str = "The host firmware is not answering."
    ) -> dict:
        """Call a backend method, turning FirmwareUnreachable into a SessionError."""
        try:
            return fn(*args)
        except FirmwareUnreachable as e:
            raise SessionError("firmware_unreachable", message) from e

    @staticmethod
    def _explain(reply: dict) -> str:
        error = reply.get("error")
        if error == "offset_out_of_range":
            servo = reply.get("servo")
            return f"Servo {servo} sits on the one position homing can't use: nudge it."
        if error == "torque_on":
            return "A joint has torque on."
        if error == "no_adapter":
            return "No USB adapter is open."
        if error == "serial_mismatch":
            return "A different USB adapter is connected now."
        return f"The servos reported: {error}"

    def _start(self, _argument: str) -> None:
        self._need_phase(IDLE)
        if not self._lock.acquire():
            self._lock_held = False
            raise SessionError("control_active", "Turn Control Robot off first.")
        self._lock_held = True
        self._phase = MID_POSE
        self._accepted = {}
        self._preset = ""

    def _capture_mid(self, _argument: str) -> None:
        self._need_phase(MID_POSE)
        self._firmware = self._call_firmware(self._backend.status)
        checks = self.health()
        if not all(c["ok"] for c in checks if c["required"]):
            failing = [c["label"] for c in checks if c["required"] and not c["ok"]]
            raise SessionError("unhealthy", f"Not ready: {'; '.join(failing)}")

        serial = self._firmware["open_serial"]
        old = self._call(self._call_firmware(self._backend.read_homing))["homing"]

        # Written first: from the next call on, this is the only copy of these values.
        self._store.write_journal({"serial": serial, "old": old, "accepted": {}})
        try:
            reply = self._backend.set_half_turn(serial)
        except FirmwareUnreachable as e:
            self._enter_recovery(serial, old)
            raise SessionError(
                "firmware_unreachable",
                "Lost the firmware during homing: restore or resume.",
            ) from e

        if not reply.get("ok"):
            if reply.get("restored", True):
                self._store.clear_journal()
            else:
                self._enter_recovery(
                    serial,
                    old,
                    "Homing failed and the previous values could not be restored.",
                )
            raise SessionError(
                str(reply.get("error") or "firmware_error"), self._explain(reply)
            )

        self._serial = serial
        self._old = reply["old"]
        self._new = reply["new"]
        self._accepted = {}
        self._mid_check_in = MID_CHECK_POLLS
        self._phase = JOINTS
        self._record_next()
        self._journal()

    def _enter_recovery(self, serial: str, old: list[dict], message: str = "") -> None:
        self._phase = RECOVERY
        self._serial = serial
        self._old = old
        if message:
            self._message = message

    def _journal(self) -> None:
        self._store.write_journal(
            {
                "serial": self._serial,
                "old": self._old,
                "new": self._new,
                "accepted": self._accepted,
                "preset": self._preset,
            }
        )

    def _begin_joint(self, name: str) -> None:
        """Record `name` from scratch: start over, or redo it from review."""
        self._need_phase(JOINTS, REVIEW)
        joint = self._joint(name)
        self._phase = JOINTS
        self._mid_check_in = (
            0  # Moving on: positions no longer say anything about homing.
        )
        self._start_recording(joint)

    def _clear_sweep(self) -> None:
        self._range = None
        self._last_tick = None

    def _start_recording(self, joint: dict) -> None:
        self._recording = joint["name"]
        self._clear_sweep()
        self._backend.begin_joint(joint["id"])

    def _record_next(self) -> None:
        """Record the first joint not yet accepted, in profile order, or go to review."""
        pending = [j for j in self._joints if j["name"] not in self._accepted]
        if pending:
            self._start_recording(pending[0])
            return
        self._recording = None
        self._clear_sweep()
        self._backend.begin_joint(-1)
        self._phase = REVIEW

    def _accept_joint(self, argument: str) -> None:
        self._need_phase(JOINTS)
        if self._recording is None or self._range is None:
            raise SessionError("nothing_recorded", "Sweep the joint first.")
        joint = self._joint(self._recording)
        low, high = self._range
        if not joint.get("full_turn") and (
            low > HALF_TURN_TICK or high < HALF_TURN_TICK
        ):
            raise SessionError(
                "mid_outside_range",
                f"{joint['name']}'s range doesn't include the mid pose.",
            )
        if high - low < joint["min_span"] and argument != "override":
            raise SessionError(
                "span_too_small",
                f"{joint['name']} only moved {high - low} ticks; expected {joint['min_span']}.",
            )
        self._accepted[joint["name"]] = (
            [0, MAX_TICK] if joint.get("full_turn") else [int(low), int(high)]
        )
        # Accept and continue: the next joint records straight away.
        self._record_next()
        self._journal()

    def lerobot_calibration(self) -> dict:
        """Return the accepted session as a LeRobot calibration file, in its homed frame."""
        offsets = {r["id"]: r["homing_offset"] for r in self._new}
        return {
            j["motor"]: {
                "id": j["id"],
                "drive_mode": 0,
                "homing_offset": offsets[j["id"]],
                "range_min": self._accepted[j["name"]][0],
                "range_max": self._accepted[j["name"]][1],
            }
            for j in self._joints
        }

    def _apply(self, argument: str) -> None:
        self._need_phase(REVIEW)
        name = argument.strip() or f"{self._prefix}_{self._serial}"
        if not PRESET_NAME.fullmatch(name) or name == TEMPLATE_PRESET:
            raise SessionError("bad_preset_name", f"Not a usable preset name: {name}")

        calibration = self.lerobot_calibration()
        try:
            validate_calibration(calibration)
            text, _limits, _warnings = apply_to_text(
                self._store.read_named_yaml(TEMPLATE_PRESET),
                calibration,
                margin_rad=MARGIN_RAD,
                full_turn_motors=[
                    j["motor"] for j in self._joints if j.get("full_turn")
                ],
            )
            config = yaml.safe_load(text)
            text = set_calibration_block(
                text,
                calibrated_at=self._clock(),
                homing_offsets=homing_offsets_by_actuator(calibration, config),
                adapter_serial=self._serial,
            )
            config = yaml.safe_load(text)
            validate_hardware_yaml(config)
            urdf_limits_from_config(config)
        except (CalibrationError, KeyError, OSError, ValueError, yaml.YAMLError) as e:
            raise SessionError(
                "apply_failed", f"Could not build the preset: {e}"
            ) from e

        self._store.write_named_yaml(name, text)
        atomic_write_text(
            self._store.lerobot_json_path(name),
            json.dumps(calibration, indent=4) + "\n",
        )
        self._preset = name
        self._phase = APPLIED
        self._journal()

    def _verify_drive(self, _argument: str) -> None:
        self._need_phase(APPLIED)
        if self._store.get_active_name() != self._preset:
            raise SessionError("not_activated", "Activate the new preset first.")
        self._lock.release()
        self._lock_held = False
        self._phase = VERIFY_DRIVE

    def _back_to_review(self, _argument: str) -> None:
        self._need_phase(APPLIED, VERIFY_DRIVE)
        if not self._lock.acquire():
            raise SessionError("control_active", "Turn Control Robot off first.")
        self._lock_held = True
        self._phase = REVIEW

    def _finish(self, _argument: str) -> None:
        self._need_phase(APPLIED, VERIFY_DRIVE)
        if self._store.get_active_name() != self._preset:
            raise SessionError("not_activated", "Activate the new preset first.")
        self._end("Calibration complete.")

    def _end(self, message: str) -> None:
        self._store.clear_journal()
        self._lock.release()
        self._lock_held = False
        self._phase = IDLE
        self._serial = ""
        self._old = []
        self._new = []
        self._accepted = {}
        self._preset = ""
        self._recording = None
        self._range = None
        self._message = message

    def _cancel(self, _argument: str) -> None:
        self._need_phase(MID_POSE, JOINTS, REVIEW, APPLIED, VERIFY_DRIVE)
        if self._phase == MID_POSE:
            self._end("")
            return
        if self._preset and self._store.get_active_name() == self._preset:
            # The active preset was built for these offsets. Putting the old ones back
            # would leave it wrong, so the new calibration stands.
            self._end("The new calibration is active, so it was kept.")
            return
        self._restore_previous()
        self._end("The previous calibration was restored.")

    def _restore(self, _argument: str) -> None:
        self._need_phase(RECOVERY)
        self._restore_previous()
        self._end("The previous calibration was restored.")

    def _restore_previous(self) -> None:
        if not self._old:
            return
        self._call(
            self._call_firmware(
                self._backend.restore_homing,
                self._serial,
                self._old,
                message="The host firmware is not answering: try again.",
            )
        )
        self._backend.begin_joint(-1)

    def _resume(self, _argument: str) -> None:
        self._need_phase(RECOVERY)
        if not self._new:
            raise SessionError(
                "cannot_resume",
                "Homing never finished, so there is nothing to resume: restore.",
            )
        found = self._call(
            self._call_firmware(
                self._backend.read_homing,
                message="The host firmware is not answering: try again.",
            )
        )["homing"]
        if found != self._new:
            raise SessionError(
                "homing_changed",
                "The servos no longer hold this session's values: restore.",
            )
        if self._preset:
            self._phase = APPLIED
        else:
            self._phase = JOINTS
            self._record_next()

    def _adopt(self, _argument: str) -> None:
        """Vouch that the active preset fits this arm, calibrated some other way."""
        self._need_phase(IDLE)
        serial = (self._firmware or {}).get("open_serial")
        if not valid_serial(serial):
            raise SessionError(
                "bad_serial", "This USB adapter has no usable serial number."
            )
        name = self._store.get_active_name()
        if name == TEMPLATE_PRESET:
            raise SessionError(
                "not_calibrated", "The stock preset is not calibrated for any arm."
            )
        self._store.bind_adapter(serial, name)
        self._message = f"{name} is now this arm's preset."

    def _select_adapter(self, serial: str) -> None:
        self._need_phase(IDLE)
        if not valid_serial(serial):
            raise SessionError("bad_serial", "Not a usable adapter serial.")
        self._call(self._call_firmware(self._backend.select, serial))
        self._store.select_adapter(serial)

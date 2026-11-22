"""Phase transitions along the wizard's forward path: start through finish."""

from __future__ import annotations

from .constants import APPLIED
from .constants import IDLE
from .constants import JOINTS
from .constants import MID_CHECK_POLLS
from .constants import MID_POSE
from .constants import RECOVERY
from .constants import REVIEW
from .constants import SessionError
from .constants import VERIFY_DRIVE
from ..firmware_client import FirmwareUnreachable


class SessionLifecycleMixin:
    """Acquire and release the control lock as the wizard moves through its phases."""

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
        failing = self.failing_checks()
        if failing:
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

        try:
            self._call(reply)
        except SessionError:
            if reply.get("restored", True):
                self._store.clear_journal()
            else:
                self._enter_recovery(
                    serial,
                    old,
                    "Homing failed and the previous values could not be restored.",
                )
            raise

        self._serial = serial
        self._old = reply["old"]
        self._new = reply["new"]
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

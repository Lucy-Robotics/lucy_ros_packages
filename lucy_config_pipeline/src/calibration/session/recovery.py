"""Ending a session, cleanly or by restoring the arm's previous calibration."""

from __future__ import annotations

from .constants import APPLIED
from .constants import IDLE
from .constants import JOINTS
from .constants import MID_POSE
from .constants import RECOVERY
from .constants import REVIEW
from .constants import SessionError
from .constants import VERIFY_DRIVE


class SessionRecoveryMixin:
    """Cancel, restore or resume a session, and the shared cleanup that ends one."""

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

    def _discard(self, _argument: str) -> None:
        self._need_phase(RECOVERY)
        self._end("The unfinished session was discarded. The servos were not touched.")

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

"""
The calibration wizard's state machine, free of ROS so it can be tested on its own.

Every EEPROM change is journaled first, so a crash can't lose the only copy of the
previous calibration.
"""

from __future__ import annotations

from .apply import SessionApplyMixin
from .constants import IDLE
from .constants import RECOVERY
from .constants import SessionError
from .constants import utc_now
from .idle_commands import SessionIdleCommandsMixin
from .lifecycle import SessionLifecycleMixin
from .recording import SessionRecordingMixin
from .recovery import SessionRecoveryMixin
from .status import SessionStatusMixin
from ..firmware_client import FirmwareUnreachable
from ...config_store import ConfigStore


class Session(
    SessionStatusMixin,
    SessionRecordingMixin,
    SessionLifecycleMixin,
    SessionRecoveryMixin,
    SessionIdleCommandsMixin,
    SessionApplyMixin,
):
    """One arm being calibrated. Callers serialize `command` and `poll`."""

    def __init__(
        self, *, backend, store: ConfigStore, profile: dict, lock, clock=utc_now
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

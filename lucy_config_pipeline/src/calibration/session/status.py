"""Reporting the session's current state: firmware polling, health checks, status."""

from __future__ import annotations

from .constants import HALF_TURN_TICK
from .constants import IDLE
from .constants import LOAD_WARN
from .constants import LOCK_REFRESH_POLLS
from .constants import MID_TOLERANCE_TICKS
from .constants import TEMPERATURE_WARN_C
from .constants import VERIFY_DRIVE
from .constants import WRAP_JUMP_TICKS
from ..firmware_client import FirmwareUnreachable
from ..identity import FRESH_READ_MS
from ..identity import servos_fresh
from ..identity import valid_serial


class SessionStatusMixin:
    """Firmware polling, the joint sweep's running range, and the panel's status payload."""

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
        lo, hi = self._range or (tick, tick)
        self._range = [min(lo, tick), max(hi, tick)]

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

    def failing_checks(self) -> list[str]:
        """Labels of every required check that is not ok."""
        return [c["label"] for c in self.health() if c["required"] and not c["ok"]]

    def healthy(self) -> bool:
        return not self.failing_checks()

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

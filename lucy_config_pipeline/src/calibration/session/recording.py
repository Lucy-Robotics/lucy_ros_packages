"""Sweeping a joint's range, accepting it, and the resulting LeRobot calibration."""

from __future__ import annotations

from .constants import HALF_TURN_TICK
from .constants import JOINTS
from .constants import MAX_TICK
from .constants import REVIEW
from .constants import SessionError


class SessionRecordingMixin:
    """Record one joint's travel at a time, in profile order, until all are accepted."""

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

"""A pretend arm for mock and CI runs. Never used with ``real:=true``."""

from __future__ import annotations

import math
import time

from .session.constants import HALF_TURN_TICK
from .session.constants import MAX_TICK

SIM_SERIAL = "SIMULATED"
SWEEP_PERIOD_S = 2.0
SWEEP_TICKS = 1300


class SimBackend:
    """
    Six servos with EEPROM homing, following the same rules as the real ones.

    Mid poses are chosen so most joints' travel crosses the encoder wrap, as they do on
    a real arm, which is what calibration has to cope with.
    """

    def __init__(self, joint_ids: list[int], clock=time.monotonic) -> None:
        self._clock = clock
        self._ids = list(joint_ids)
        self._mid = {sid: (3700 + 700 * i) % 4096 for i, sid in enumerate(self._ids)}
        self._records = {
            sid: {"id": sid, "homing_offset": 0, "min_limit": 0, "max_limit": MAX_TICK}
            for sid in self._ids
        }
        self._sweeping: int | None = None
        self._sweep_start = 0.0

    def _actual(self, sid: int) -> int:
        mid = self._mid[sid]
        if sid != self._sweeping:
            return mid
        phase = 2 * math.pi * (self._clock() - self._sweep_start) / SWEEP_PERIOD_S
        return round(mid + SWEEP_TICKS * math.sin(phase)) % 4096

    def _present(self, sid: int) -> int:
        return (self._actual(sid) - self._records[sid]["homing_offset"]) % 4096

    def status(self) -> dict:
        return {
            "ok": True,
            "link": "open",
            "adapters": [{"port": "/dev/null", "serial": SIM_SERIAL}],
            "selected": None,
            "open_serial": SIM_SERIAL,
            "open_error": None,
            "servos": [
                {
                    "id": sid,
                    "ticks": self._present(sid),
                    "temperature": 30,
                    "load": 0,
                    "age_ms": 0,
                    "torque": False,
                }
                for sid in self._ids
            ],
        }

    def select(self, _serial: str) -> dict:
        return self.status()

    def _record_list(self) -> list[dict]:
        return [dict(self._records[sid]) for sid in self._ids]

    def read_homing(self) -> dict:
        return {"ok": True, "homing": self._record_list()}

    def set_half_turn(self, serial: str) -> dict:
        if serial != SIM_SERIAL:
            return {"ok": False, "error": "serial_mismatch"}
        old = self._record_list()
        self._sweeping = None
        for sid in self._ids:
            self._records[sid] = {
                "id": sid,
                "homing_offset": self._mid[sid] - HALF_TURN_TICK,
                "min_limit": 0,
                "max_limit": MAX_TICK,
            }
        return {"ok": True, "old": old, "new": self._record_list()}

    def restore_homing(self, serial: str, records: list[dict]) -> dict:
        if serial != SIM_SERIAL:
            return {"ok": False, "error": "serial_mismatch"}
        self._sweeping = None
        for record in records:
            self._records[record["id"]] = dict(record)
        return {"ok": True}

    def begin_joint(self, joint_id: int) -> None:
        self._sweeping = joint_id
        self._sweep_start = self._clock()

    def close(self) -> None:
        """Nothing to release."""

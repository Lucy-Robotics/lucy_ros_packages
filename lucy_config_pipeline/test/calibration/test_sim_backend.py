"""The sim backend, standing in for the firmware client in mock and CI runs."""

from __future__ import annotations

import math

from src.calibration.sim_backend import HALF_TURN_TICK
from src.calibration.sim_backend import MAX_TICK
from src.calibration.sim_backend import SIM_SERIAL
from src.calibration.sim_backend import SimBackend
from src.calibration.sim_backend import SWEEP_PERIOD_S
from src.calibration.sim_backend import SWEEP_TICKS


class FakeClock:
    def __init__(self, start: float = 0.0) -> None:
        self.now = start

    def __call__(self) -> float:
        return self.now


def test_status_reports_every_joint_at_rest():
    backend = SimBackend([1, 2, 3], clock=FakeClock())
    status = backend.status()
    assert status["ok"] is True
    assert status["open_serial"] == SIM_SERIAL
    assert [s["id"] for s in status["servos"]] == [1, 2, 3]
    assert all(s["torque"] is False for s in status["servos"])


def test_select_returns_the_same_shape_as_status():
    backend = SimBackend([1])
    assert backend.select(SIM_SERIAL) == backend.status()


def test_read_homing_starts_at_zero_offset_full_range():
    backend = SimBackend([1, 2])
    homing = backend.read_homing()
    assert homing == {
        "ok": True,
        "homing": [
            {"id": 1, "homing_offset": 0, "min_limit": 0, "max_limit": MAX_TICK},
            {"id": 2, "homing_offset": 0, "min_limit": 0, "max_limit": MAX_TICK},
        ],
    }


def test_set_half_turn_rejects_the_wrong_serial():
    backend = SimBackend([1])
    assert backend.set_half_turn("other") == {"ok": False, "error": "serial_mismatch"}


def test_set_half_turn_recenters_every_joint_on_the_half_turn_tick():
    backend = SimBackend([1, 2])
    result = backend.set_half_turn(SIM_SERIAL)
    assert result["ok"] is True
    for sid, record in zip([1, 2], result["new"], strict=True):
        assert record["homing_offset"] == backend._mid[sid] - HALF_TURN_TICK
        assert record["min_limit"] == 0
        assert record["max_limit"] == MAX_TICK


def test_set_half_turn_stops_any_in_progress_sweep():
    clock = FakeClock()
    backend = SimBackend([1], clock=clock)
    backend.begin_joint(1)
    backend.set_half_turn(SIM_SERIAL)

    at_rest = backend.status()["servos"][0]["ticks"]
    clock.now = 10.0
    assert backend.status()["servos"][0]["ticks"] == at_rest


def test_restore_homing_rejects_the_wrong_serial():
    backend = SimBackend([1])
    result = backend.restore_homing("other", [])
    assert result == {"ok": False, "error": "serial_mismatch"}


def test_restore_homing_writes_back_the_given_records():
    backend = SimBackend([1, 2])
    records = [
        {"id": 1, "homing_offset": -5, "min_limit": 10, "max_limit": 4000},
        {"id": 2, "homing_offset": 7, "min_limit": 20, "max_limit": 3900},
    ]
    result = backend.restore_homing(SIM_SERIAL, records)
    assert result == {"ok": True}
    assert backend.read_homing()["homing"] == records


def test_begin_joint_sweeps_only_the_selected_joint():
    clock = FakeClock()
    backend = SimBackend([1, 2], clock=clock)
    backend.begin_joint(1)

    clock.now = 0.5
    phase = 2 * math.pi * clock.now / SWEEP_PERIOD_S
    expected = round(backend._mid[1] + SWEEP_TICKS * math.sin(phase)) % 4096

    servos = {s["id"]: s["ticks"] for s in backend.status()["servos"]}
    assert servos[1] == expected
    assert servos[2] == backend._mid[2]


def test_close_does_not_raise():
    SimBackend([1]).close()

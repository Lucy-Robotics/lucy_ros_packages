"""Helpers for driving a session through a sweep, shared by the session test files."""

from __future__ import annotations

import pytest
from conftest import Arm
from src.calibration.session import Session
from src.calibration.session import SessionError
from src.calibration.sim_backend import SWEEP_PERIOD_S


def sweep(session: Session, arm: Arm, joint: str) -> None:
    session.command("begin_joint", joint)
    for step in range(41):
        arm.clock.now += SWEEP_PERIOD_S / 40
        session.poll()
    assert step == 40


def run_sweep(session: Session, arm: Arm) -> None:
    for _ in range(41):
        arm.clock.now += SWEEP_PERIOD_S / 40
        session.poll()


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

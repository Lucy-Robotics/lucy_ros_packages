"""Helpers for driving a session through a sweep, shared by the session test files."""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

from conftest import Arm
import pytest
from src.calibration.session import Session
from src.calibration.session import SessionError
from src.calibration.sim_backend import SWEEP_PERIOD_S

SWEEP_STEPS = 40


def sweep(session: Session, arm: Arm, joint: str) -> None:
    session.command("begin_joint", joint)
    run_sweep(session, arm)


def run_sweep(session: Session, arm: Arm) -> None:
    for _ in range(SWEEP_STEPS + 1):
        arm.clock.now += SWEEP_PERIOD_S / SWEEP_STEPS
        session.poll()


def calibrate_all(session: Session, arm: Arm) -> None:
    session.command("start")
    session.command("capture_mid")
    for joint in ("Rotation", "Pitch", "Wrist_Roll", "Jaw"):
        sweep(session, arm, joint)
        session.command("accept_joint")


def apply_all(session: Session, arm: Arm) -> None:
    calibrate_all(session, arm)
    session.command("apply")


def reborn_session(arm: Arm, store: Any, profile: dict, lock: Any) -> Session:
    return Session(backend=arm, store=store, profile=profile, lock=lock)


def patch_status(arm: Arm, mutate: Callable[[dict], None]) -> None:
    """Make `arm.status()` return a reading tampered with by `mutate`, in place."""
    real = arm.status

    def tampered() -> dict:
        status = real()
        mutate(status)
        return status

    arm.status = tampered


def code_of(session: Session, command: str, argument: str = "") -> str:
    with pytest.raises(SessionError) as e:
        session.command(command, argument)
    return e.value.code

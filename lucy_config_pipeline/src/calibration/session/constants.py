"""Phase names, tuning constants, and the session-level error type."""

from __future__ import annotations

import math
import re
from datetime import datetime
from datetime import timezone

import yaml

from ...config_store import ConfigStore

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


def utc_now() -> str:
    return (
        datetime.now(timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z")
    )

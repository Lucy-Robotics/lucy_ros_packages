"""
The calibration wizard's state machine, free of ROS so it can be tested on its own.

Every EEPROM change is journaled first, so a crash can't lose the only copy of the
previous calibration.
"""

from __future__ import annotations

from .constants import APPLIED
from .constants import HALF_TURN_TICK
from .constants import IDLE
from .constants import JOINTS
from .constants import LOAD_WARN
from .constants import LOCK_REFRESH_POLLS
from .constants import MARGIN_RAD
from .constants import MAX_TICK
from .constants import MID_CHECK_POLLS
from .constants import MID_POSE
from .constants import MID_TOLERANCE_TICKS
from .constants import PRESET_NAME
from .constants import RECOVERY
from .constants import REVIEW
from .constants import SessionError
from .constants import TEMPERATURE_WARN_C
from .constants import TEMPLATE_PRESET
from .constants import VERIFY_DRIVE
from .constants import WRAP_JUMP_TICKS
from .constants import load_profile
from .core import Session

__all__ = [
    "APPLIED",
    "HALF_TURN_TICK",
    "IDLE",
    "JOINTS",
    "load_profile",
    "LOAD_WARN",
    "LOCK_REFRESH_POLLS",
    "MARGIN_RAD",
    "MAX_TICK",
    "MID_CHECK_POLLS",
    "MID_POSE",
    "MID_TOLERANCE_TICKS",
    "PRESET_NAME",
    "RECOVERY",
    "REVIEW",
    "Session",
    "SessionError",
    "TEMPERATURE_WARN_C",
    "TEMPLATE_PRESET",
    "VERIFY_DRIVE",
    "WRAP_JUMP_TICKS",
]

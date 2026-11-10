# Copyright 2026 Lucy Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

r"""
Convert between raw encoder ticks and the radians Lucy's hardware YAML stores.

LeRobot's half-turn homing makes the pose held during calibration read as tick
2048 (π rad on Lucy's 0–2π ↔ 0–4096 map). That pose is the URDF default (joint
0).
"""

from __future__ import annotations

import math

from lucy_config_generator.calibration.errors import CalibrationError

# Firmware / YAML map 0-2pi rad linearly onto the STS3215 0-4096 tick span.
TICKS_PER_TURN = 4096
# LeRobot half-turn homing centres Present_Position here (= URDF joint 0).
CENTRE_TICKS = TICKS_PER_TURN // 2


def ticks_to_radians(ticks: float) -> float:
    """Convert raw encoder ticks to radians stored in the hardware YAML."""
    return round(ticks * (2.0 * math.pi) / TICKS_PER_TURN, 6)


def servo_window_to_joint_limits(
    limit_min: float,
    limit_max: float,
    offset: float,
    direction: float,
    scale: float,
) -> tuple[float, float]:
    """Map a servo-frame travel window to URDF joint lower/upper (sorted)."""
    denom = direction * scale
    if denom == 0:
        raise CalibrationError("direction*scale must be non-zero")
    a = (limit_min - offset) * denom
    b = (limit_max - offset) * denom
    lo, hi = (a, b) if a <= b else (b, a)
    return round(lo, 6), round(hi, 6)


def window_around_centre(
    limit_min: float,
    limit_max: float,
    centre: float,
    margin: float = 0.0,
) -> tuple[float, float]:
    """
    Inset a travel window by ``margin`` while always keeping ``centre`` inside it.

    The schema requires ``default`` (the centre) within the limits. A joint can
    rest against its own stop at the mid pose, as the jaw does, and homing lands
    on tick 2047 while π is tick 2048.
    """
    return min(limit_min + margin, centre), max(limit_max - margin, centre)

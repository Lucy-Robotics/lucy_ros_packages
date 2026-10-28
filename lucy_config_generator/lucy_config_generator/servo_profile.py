# Copyright 2026 Lucy Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

"""Servo travel and default pulse ends. The MCU never sees ``servo_type``."""

from __future__ import annotations

import math

MIN_PULSE_US = 500
MAX_PULSE_US = 2500

_AMPLITUDE = {
    '180': math.pi,
    '270': 1.5 * math.pi,
    '300': 5.0 * math.pi / 3.0,
}


def amplitude_rad(servo_type: str) -> float:
    """
    Full-scale travel in radians for a ``servo_type`` of 180, 270, or 300.

    :param servo_type: Hardware YAML servo type, quotes already stripped.
    :raises ValueError: The type is not one of the three known travels.
    """
    key = str(servo_type).strip().strip('"')
    try:
        return _AMPLITUDE[key]
    except KeyError as exc:
        raise ValueError(f'invalid servo_type {servo_type!r}') from exc

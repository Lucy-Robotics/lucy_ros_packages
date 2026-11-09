# Copyright 2026 Lucy Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

r"""
Turn a LeRobot-shaped servo calibration into hardware YAML and URDF limits.

LeRobot's half-turn homing makes the pose held during calibration read as tick
2048 (π rad on Lucy's 0–2π ↔ 0–4096 map). That pose is the URDF default (joint
0). Lucy therefore stores:

* ``offset`` / ``default`` = π (servo-frame centre)
* ``limit_min`` / ``limit_max`` = travel window in servo-frame radians
* URDF ``lower`` / ``upper`` = the same window mapped to joint space:
  ``joint = (servo - offset) * direction * scale``
"""

from __future__ import annotations

from lucy_config_generator.calibration.errors import CalibrationError
from lucy_config_generator.calibration.planning import plan
from lucy_config_generator.calibration.planning import sensor_updates
from lucy_config_generator.calibration.preset import generated_limit_mismatches
from lucy_config_generator.calibration.preset import homing_offsets_by_actuator
from lucy_config_generator.calibration.preset import LIMIT_TOLERANCE_RAD
from lucy_config_generator.calibration.preset import set_calibration_block
from lucy_config_generator.calibration.preset import urdf_limits_from_config
from lucy_config_generator.calibration.rewrite import apply_to_file
from lucy_config_generator.calibration.rewrite import apply_to_text
from lucy_config_generator.calibration.rewrite import apply_updates
from lucy_config_generator.calibration.rewrite import apply_urdf_limits
from lucy_config_generator.calibration.ticks import CENTRE_TICKS
from lucy_config_generator.calibration.ticks import servo_window_to_joint_limits
from lucy_config_generator.calibration.ticks import TICKS_PER_TURN
from lucy_config_generator.calibration.ticks import ticks_to_radians
from lucy_config_generator.calibration.ticks import window_around_centre
from lucy_config_generator.calibration.validation import KNOWN_DRIVE_MODES
from lucy_config_generator.calibration.validation import load_calibration
from lucy_config_generator.calibration.validation import REQUIRED_RECORD_KEYS
from lucy_config_generator.calibration.validation import validate_calibration

__all__ = [
    "CalibrationError",
    "plan",
    "sensor_updates",
    "generated_limit_mismatches",
    "homing_offsets_by_actuator",
    "set_calibration_block",
    "urdf_limits_from_config",
    "LIMIT_TOLERANCE_RAD",
    "apply_to_file",
    "apply_to_text",
    "apply_updates",
    "apply_urdf_limits",
    "CENTRE_TICKS",
    "servo_window_to_joint_limits",
    "ticks_to_radians",
    "TICKS_PER_TURN",
    "window_around_centre",
    "KNOWN_DRIVE_MODES",
    "load_calibration",
    "REQUIRED_RECORD_KEYS",
    "validate_calibration",
]

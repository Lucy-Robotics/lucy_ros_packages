# Copyright 2026 Lucy Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

"""Shared calibration test data, including values from a real SO-ARM101."""

from __future__ import annotations

from lucy_config_generator.calibration import ticks_to_radians

# From lerobot-calibrate on a real arm.
REAL_ARM = {
    "shoulder_pan": {
        "id": 1,
        "drive_mode": 0,
        "homing_offset": 1643,
        "range_min": 753,
        "range_max": 3172,
    },
    "shoulder_lift": {
        "id": 2,
        "drive_mode": 0,
        "homing_offset": -1154,
        "range_min": 822,
        "range_max": 3248,
    },
    "elbow_flex": {
        "id": 3,
        "drive_mode": 0,
        "homing_offset": 1124,
        "range_min": 841,
        "range_max": 3072,
    },
    "wrist_flex": {
        "id": 4,
        "drive_mode": 0,
        "homing_offset": 1799,
        "range_min": 853,
        "range_max": 3195,
    },
    "wrist_roll": {
        "id": 5,
        "drive_mode": 0,
        "homing_offset": 446,
        "range_min": 0,
        "range_max": 4095,
    },
    "gripper": {
        "id": 6,
        "drive_mode": 0,
        "homing_offset": 1389,
        "range_min": 2036,
        "range_max": 3541,
    },
}
NAMES = ["rotation", "pitch", "elbow", "wrist_pitch", "wrist_roll", "jaw"]
CONFIG = {
    "actuators": [
        {
            "id": name,
            "urdf_joint": name.title(),
            "physical_pin": i + 1,
            "direction": 1,
            "scale": 1,
        }
        for i, name in enumerate(NAMES)
    ]
}
CENTRE = ticks_to_radians(2048)

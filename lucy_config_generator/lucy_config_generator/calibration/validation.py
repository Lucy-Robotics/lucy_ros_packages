# Copyright 2026 Lucy Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

"""Reject a LeRobot calibration that cannot be applied to the config."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from lucy_config_generator.calibration.errors import CalibrationError

REQUIRED_RECORD_KEYS = ("id", "range_min", "range_max")
KNOWN_DRIVE_MODES = (0, 1)


def validate_calibration(data: Any, source: str = "calibration") -> dict[str, dict]:
    """Reject records that cannot be applied, including an unknown ``drive_mode``."""
    if not isinstance(data, dict) or not data:
        raise CalibrationError(
            f"{source}: expected a non-empty object of motor records"
        )

    for motor, record in data.items():
        if not isinstance(record, dict):
            raise CalibrationError(f"{source}: {motor!r} is not an object")
        missing = [k for k in REQUIRED_RECORD_KEYS if k not in record]
        if missing:
            raise CalibrationError(f"{source}: {motor!r} is missing {missing}")
        if record["range_min"] >= record["range_max"]:
            raise CalibrationError(
                f"{source}: {motor!r} has range_min {record['range_min']} "
                f">= range_max {record['range_max']}"
            )
        drive_mode = record.get("drive_mode", 0)
        if drive_mode not in KNOWN_DRIVE_MODES:
            raise CalibrationError(
                f"{source}: {motor!r} has unknown drive_mode {drive_mode!r}; "
                f"expected one of {KNOWN_DRIVE_MODES}"
            )
    return data


def load_calibration(path: Path) -> dict[str, dict]:
    """Read a lerobot-calibrate JSON, rejecting records that cannot be applied."""
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as e:
        raise CalibrationError(f"cannot read calibration {path}: {e}") from e
    return validate_calibration(data, str(path))

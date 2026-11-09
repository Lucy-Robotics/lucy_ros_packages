# Copyright 2026 Lucy Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

"""Pair a LeRobot calibration's records with config actuators and derive values."""

from __future__ import annotations

from typing import Iterable

from lucy_config_generator.calibration.errors import CalibrationError
from lucy_config_generator.calibration.ticks import CENTRE_TICKS
from lucy_config_generator.calibration.ticks import servo_window_to_joint_limits
from lucy_config_generator.calibration.ticks import ticks_to_radians
from lucy_config_generator.calibration.ticks import window_around_centre


def _actuators_by_pin(actuators: Iterable[dict]) -> dict[int, list[dict]]:
    """Group actuators by ``physical_pin`` (the servo id LeRobot records against)."""
    by_pin: dict[int, list[dict]] = {}
    for actuator in actuators:
        pin = actuator.get("physical_pin")
        if pin is not None:
            by_pin.setdefault(int(pin), []).append(actuator)
    return by_pin


def plan(
    calibration: dict[str, dict],
    config: dict,
    *,
    margin_rad: float = 0.0,
    full_turn_motors: Iterable[str] = (),
) -> tuple[dict, dict[str, tuple[float, float]], list[str]]:
    """
    Pair calibration records with actuators by servo id, and derive their values.

    Returns per-actuator YAML field updates, per-``urdf_joint`` limit pairs, and
    warnings. Raises when a record matches no actuator or two actuators share
    its id.

    ``margin_rad`` keeps joints off the stops just recorded. Motors in
    ``full_turn_motors`` have no stops, so they get no margin.
    """
    actuators = config.get("actuators") or []
    full_turn = set(full_turn_motors)
    warnings: list[str] = []
    updates: dict[str, dict[str, float]] = {}
    urdf_limits: dict[str, tuple[float, float]] = {}

    by_pin = _actuators_by_pin(actuators)

    centre = ticks_to_radians(CENTRE_TICKS)
    for motor, record in sorted(calibration.items(), key=lambda kv: kv[1]["id"]):
        servo_id = int(record["id"])
        matched = by_pin.get(servo_id, [])
        if not matched:
            raise CalibrationError(
                f"{motor!r} (servo id {servo_id}) matches no actuator: no "
                f"physical_pin {servo_id} in the config"
            )
        if len(matched) > 1:
            ids = [str(a["id"]) for a in matched]
            raise CalibrationError(
                f"{motor!r} (servo id {servo_id}) matches several actuators "
                f"({', '.join(ids)}); physical_pin must be unique"
            )
        actuator = matched[0]
        actuator_id = str(actuator["id"])
        if record.get("drive_mode"):
            warnings.append(
                f"{motor}: drive_mode={record['drive_mode']} means LeRobot drives "
                f"this joint inverted. direction is left at its current value; "
                f"check {actuator_id} against the 3D view."
            )
        limit_min, limit_max = window_around_centre(
            ticks_to_radians(record["range_min"]),
            ticks_to_radians(record["range_max"]),
            centre,
            0.0 if motor in full_turn else margin_rad,
        )
        updates[actuator_id] = {
            "offset": centre,
            "limit_min": limit_min,
            "limit_max": limit_max,
            "default": centre,
        }

        urdf_joint = str(actuator.get("urdf_joint") or "").strip()
        if urdf_joint:
            urdf_limits[urdf_joint] = servo_window_to_joint_limits(
                limit_min,
                limit_max,
                centre,
                float(actuator.get("direction", 1)),
                float(actuator.get("scale", 1)),
            )

    unmatched = [str(a["id"]) for a in actuators if str(a["id"]) not in updates]
    if unmatched:
        warnings.append(
            f"no calibration record for: {', '.join(sorted(unmatched))} "
            f"(left untouched)"
        )
    return updates, urdf_limits, warnings


def sensor_updates(config: dict, updates: dict[str, dict]) -> dict[str, dict]:
    """Mirror each actuator's new window onto the sensors that report on it."""
    out: dict[str, dict[str, float]] = {}
    for sensor in config.get("sensors") or []:
        actuator = str(sensor.get("associated_actuator", ""))
        if actuator in updates:
            out[str(sensor["id"])] = {
                "min_value": updates[actuator]["limit_min"],
                "max_value": updates[actuator]["limit_max"],
            }
    return out

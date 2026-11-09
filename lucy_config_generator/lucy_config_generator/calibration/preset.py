# Copyright 2026 Lucy Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

"""The persisted ``calibration:`` block that ties a preset to one arm's EEPROM."""

from __future__ import annotations

import re

import yaml

from lucy_config_generator.calibration.planning import _actuators_by_pin
from lucy_config_generator.calibration.ticks import servo_window_to_joint_limits

# Generated values are written with six significant digits.
LIMIT_TOLERANCE_RAD = 1e-3


def homing_offsets_by_actuator(
    calibration: dict[str, dict], config: dict
) -> dict[str, int]:
    """Each actuator's ``homing_offset``, keyed by actuator id (records without one skipped)."""
    by_pin = _actuators_by_pin(config.get("actuators") or [])
    return {
        str(by_pin[int(record["id"])][0]["id"]): int(record["homing_offset"])
        for record in calibration.values()
        if "homing_offset" in record and int(record["id"]) in by_pin
    }


def set_calibration_block(
    text: str,
    *,
    calibrated_at: str,
    homing_offsets: dict[str, int],
    adapter_serial: str = "",
) -> str:
    """
    Add or replace the root ``calibration:`` block, leaving the rest of the text alone.

    Its presence is what ties a preset's limits to the EEPROM of one arm: activating
    the preset applies them to the URDF. ``adapter_serial`` is empty when calibrated
    outside the wizard, which has no way to know it.
    """
    block = yaml.safe_dump(
        {
            "calibration": {
                "adapter_serial": adapter_serial,
                "calibrated_at": calibrated_at,
                "homing_offsets": homing_offsets,
            }
        },
        sort_keys=False,
    )
    stripped = re.sub(
        r"(?m)^calibration:[^\n]*\n(?:[ \t]+[^\n]*\n|[ \t]*\n)*", "", text
    )
    return stripped.rstrip("\n") + "\n\n" + block


def urdf_limits_from_config(config: dict) -> dict[str, tuple[float, float]]:
    """Joint-space limits every mapped actuator's travel window implies."""
    limits: dict[str, tuple[float, float]] = {}
    for actuator in config.get("actuators") or []:
        joint = str(actuator.get("urdf_joint") or "").strip()
        if joint:
            limits[joint] = servo_window_to_joint_limits(
                float(actuator["limit_min"]),
                float(actuator["limit_max"]),
                float(actuator["offset"]),
                float(actuator.get("direction", 1)),
                float(actuator.get("scale", 1)),
            )
    return limits


def generated_limit_mismatches(xacro_text: str, config: dict) -> list[str]:
    """Describe where a generated ros2_control xacro disagrees with this config."""
    problems: list[str] = []
    for actuator in config.get("actuators") or []:
        joint = str(actuator.get("urdf_joint") or "").strip()
        if not joint:
            continue
        section = re.search(
            rf'<joint name="{re.escape(joint)}">(.*?)</joint>', xacro_text, re.S
        )
        if section is None:
            problems.append(f"{joint}: not in the generated xacro")
            continue
        lower, upper = urdf_limits_from_config({"actuators": [actuator]})[joint]
        expected = {
            "offset": float(actuator["offset"]),
            "limit_min": float(actuator["limit_min"]),
            "limit_max": float(actuator["limit_max"]),
            "min": lower,
            "max": upper,
        }
        for name, wanted in expected.items():
            found = re.search(
                rf'<param name="{name}">([^<]+)</param>', section.group(1)
            )
            if found is None:
                problems.append(f"{joint}: no {name} param")
            elif abs(float(found.group(1)) - wanted) > LIMIT_TOLERANCE_RAD:
                problems.append(
                    f"{joint}: {name} is {found.group(1)}, expected {wanted:g}"
                )
    return problems

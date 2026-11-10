# Copyright 2026 Lucy Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

from __future__ import annotations

from functools import cache
from pathlib import Path
from typing import Any

import yaml

CATALOGUE_PATH = Path(__file__).with_name("motors.yaml")


@cache
def catalogue() -> dict[str, Any]:
    return yaml.safe_load(CATALOGUE_PATH.read_text())


def motor_spec(ref: str) -> dict[str, Any]:
    motor_type, _, variant = ref.partition("/")
    family = catalogue().get(motor_type)
    variants = (family or {}).get("variants", {})
    if variant not in variants:
        raise KeyError(f"unknown motor {ref!r}")
    shared = {k: v for k, v in family.items() if k != "variants"}
    return {**shared, **variants[variant]}


def motors_used(config: dict[str, Any]) -> dict[str, dict[str, Any]]:
    refs = {a["motor"] for a in config.get("actuators", []) if a.get("motor")}
    return {ref: motor_spec(ref) for ref in sorted(refs)}


def motor_values_by_joint(config: dict[str, Any], key: str) -> dict[str, float]:
    values = {}
    for actuator in config.get("actuators", []):
        ref = actuator.get("motor")
        if ref and key in (spec := motor_spec(ref)):
            values[actuator["urdf_joint"]] = float(spec[key])
    return values

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

from lucy_config_generator.generate import urdf_joint_names
from lucy_config_generator.schema import URDF_IGNORE_LIST_KEYS
from lucy_config_generator.schema import URDF_PASSIVE_LIST_KEYS
from lucy_config_generator.schema import validate_hardware_yaml
import yaml


@dataclass(frozen=True)
class ValidationReport:
    errors: list[str]
    warnings: list[str]


def parse_yaml_text(config_yaml: str) -> dict[str, Any]:
    data = yaml.safe_load(config_yaml)
    if not isinstance(data, dict):
        raise ValueError('YAML root must be a mapping')
    return data


def validate_schema(config_yaml: str) -> dict[str, Any]:
    data = parse_yaml_text(config_yaml)
    validate_hardware_yaml(data)
    return data


def urdf_crosscheck(
    data: dict[str, Any],
    urdf_xacro: Path,
    base_path: Path,
    controller_config: Path,
) -> ValidationReport:
    errors: list[str] = []
    warnings: list[str] = []
    joints = urdf_joint_names(urdf_xacro, base_path, controller_config)

    actuated = set()
    for a in data.get('actuators', []):
        j = str(a.get('urdf_joint', '')).strip()
        if not j:
            continue
        actuated.add(j)
        if j not in joints:
            errors.append(f"actuator {a.get('id')}: urdf_joint {j!r} not in URDF")

    passive_ignore = set()
    for key in URDF_PASSIVE_LIST_KEYS + URDF_IGNORE_LIST_KEYS:
        for x in data.get(key) or []:
            if isinstance(x, str) and x.strip():
                passive_ignore.add(x.strip())

    for j in sorted(joints - actuated - passive_ignore):
        warnings.append(f'URDF joint {j!r} is not mapped to any actuator')

    return ValidationReport(errors=errors, warnings=warnings)

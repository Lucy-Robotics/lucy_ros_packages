# Copyright 2026 Lucy Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

import os
from pathlib import Path

import yaml

CONTROLLERS_YAML = "controllers.yaml"


def robot_source_root(package: str, pkg_share: str) -> Path | None:
    cwd_candidate = Path.cwd() / "src" / package
    if cwd_candidate.is_dir():
        return cwd_candidate
    for parent in Path(pkg_share).parents:
        if parent.name == "install":
            src_candidate = parent.parent / "src" / package
            if src_candidate.is_dir():
                return src_candidate
    return None


def active_preset(package: str, pkg_share: str) -> dict:
    candidates = []
    source_root = robot_source_root(package, pkg_share)
    if source_root is not None:
        candidates.append(source_root / "config" / "hardware" / "active.yaml")
    candidates.append(Path(pkg_share) / "config" / "hardware" / "active.yaml")
    for active in candidates:
        if not active.is_file():
            continue
        try:
            data = yaml.safe_load(active.read_text(encoding="utf-8"))
        except (OSError, yaml.YAMLError):
            return {}
        return data if isinstance(data, dict) else {}
    return {}


def generated_files(active: dict, ros2_control_xacro: str) -> dict[str, str]:
    names = {
        "ros2_control_xacro": ros2_control_xacro,
        "controllers_yaml": CONTROLLERS_YAML,
    }
    section = active.get("generated_files")
    if isinstance(section, dict):
        for key in names:
            value = section.get(key)
            if isinstance(value, str) and value.strip() and "/" not in value:
                names[key] = value.strip()
    return names


def default_controllers_yaml(package: str, pkg_share: str, basename: str) -> str:
    source_root = robot_source_root(package, pkg_share)
    if source_root is not None:
        candidate = source_root / "config" / basename
        if candidate.is_file():
            return str(candidate.resolve())
    return os.path.join(pkg_share, "config", basename)

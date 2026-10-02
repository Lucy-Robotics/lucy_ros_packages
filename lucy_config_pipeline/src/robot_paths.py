"""Resolve robot package paths from ``config/control.launch.yaml`` when present."""

from __future__ import annotations

from pathlib import Path

import yaml

# Fallback when ``config/control.launch.yaml`` is absent (prefer that file).
_DEFAULT_URDF_REL = 'description/urdf/robot.urdf.xacro'
_DEFAULT_BASE_REL = 'description'
_DEFAULT_CONTROLLERS_REL = 'config/controllers.yaml'


def load_robot_launch_defaults(robot_root: Path) -> dict[str, str]:
    """Relative path defaults from ``config/control.launch.yaml`` when present."""
    config_path = robot_root / 'config' / 'control.launch.yaml'
    if not config_path.is_file():
        return {}
    try:
        data = yaml.safe_load(config_path.read_text(encoding='utf-8')) or {}
    except (OSError, yaml.YAMLError):
        return {}
    if not isinstance(data, dict):
        return {}
    out: dict[str, str] = {}
    for key in ('urdf_path', 'base_path', 'controllers_yaml'):
        value = data.get(key)
        if isinstance(value, str) and value.strip():
            out[key] = value.strip()
    return out


def resolve_under_robot_root(robot_root: Path, rel_or_abs: str) -> Path:
    """Resolve a launch-default path against the robot package root."""
    p = Path(rel_or_abs)
    if p.is_absolute():
        return p.resolve()
    return (robot_root / p).resolve()


def resolve_robot_description_paths(
    robot_root: Path,
    *,
    controllers_basename: str | None = None,
) -> tuple[Path, Path, Path]:
    """
    Return ``(urdf_xacro, base_path, controllers_yaml)`` for a robot package.

    Prefers ``config/control.launch.yaml``. Falls back to
    ``description/urdf/robot.urdf.xacro``. When ``controllers_basename`` is set
    (from ``generated_files``), it overrides the controllers filename while
    keeping the directory from the launch default / fallback.
    """
    defaults = load_robot_launch_defaults(robot_root)
    urdf = resolve_under_robot_root(
        robot_root, defaults.get('urdf_path', _DEFAULT_URDF_REL)
    )
    base = resolve_under_robot_root(
        robot_root, defaults.get('base_path', _DEFAULT_BASE_REL)
    )
    controllers_rel = defaults.get('controllers_yaml', _DEFAULT_CONTROLLERS_REL)
    if controllers_basename:
        controllers_rel = str(Path(controllers_rel).with_name(controllers_basename))
    controllers = resolve_under_robot_root(robot_root, controllers_rel)
    return urdf, base, controllers

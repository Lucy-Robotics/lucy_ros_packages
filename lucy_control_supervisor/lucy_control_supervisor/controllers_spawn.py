"""Shared helper: controller names declared in controllers.yaml."""

from __future__ import annotations

from pathlib import Path

import yaml

TORQUE_ENABLE_INTERFACE = 'torque_enable'


def _load(controllers_yaml_path: Path) -> dict:
    data = yaml.safe_load(controllers_yaml_path.read_text(encoding='utf-8')) or {}
    if not isinstance(data, dict):
        return {}
    return data


def torque_enable_controllers(controllers_yaml_path: Path) -> list[tuple[str, int]]:
    """Return ``(name, joint count)`` for controllers bound to ``torque_enable``."""
    found: list[tuple[str, int]] = []
    for name, body in _load(controllers_yaml_path).items():
        if not isinstance(body, dict):
            continue
        params = body.get('ros__parameters')
        if not isinstance(params, dict):
            continue
        if params.get('interface_name') != TORQUE_ENABLE_INTERFACE:
            continue
        joints = params.get('joints') or []
        found.append((str(name), len(joints)))
    return found


def torque_command_values(enabled: bool, joint_count: int) -> list[float]:
    """One ``torque_enable`` command per joint: 1.0 when Control Robot is ON."""
    value = 1.0 if enabled else 0.0
    return [value] * joint_count


def torque_publishers_matched(pubs: dict[str, tuple]) -> bool:
    """Return whether every commands publisher has at least one subscription."""
    if not pubs:
        return True
    return all(pub.get_subscription_count() > 0 for pub, _count in pubs.values())


def wait_for_torque_match(pubs: dict[str, tuple], wait_for, timeout_s: float) -> bool:
    """
    Poll until torque command publishers are matched, or ``timeout_s`` elapses.

    ``wait_for`` is the supervisor's bounded poll helper. Returns whether every
    publisher reported a subscription before the deadline.
    """
    if not pubs:
        return True
    return bool(wait_for(lambda: torque_publishers_matched(pubs), timeout_s))


def controllers_to_spawn(
    controllers_yaml_path: Path,
    *,
    include_torque_enable: bool = False,
) -> list[str]:
    """
    Return controller names under controller_manager.ros__parameters.

    Skips ``update_rate``. Gazebo launches call this with the default and omit
    ``torque_enable`` controllers: ``GazeboSimSystem`` does not export that
    interface. The real supervisor passes ``include_torque_enable=True``.
    """
    data = _load(controllers_yaml_path)
    cm_params = data.get('controller_manager', {}).get('ros__parameters', {})
    if not isinstance(cm_params, dict):
        return []
    names = [name for name in cm_params.keys() if name != 'update_rate']
    if not include_torque_enable:
        torque_names = {name for name, _count in torque_enable_controllers(controllers_yaml_path)}
        names = [name for name in names if name not in torque_names]
    jsb = 'joint_state_broadcaster'
    if jsb in names:
        names.remove(jsb)
        return [jsb, *names]
    return names

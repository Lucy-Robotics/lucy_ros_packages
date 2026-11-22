"""Pure helpers for the client registry's servo-power check."""

from __future__ import annotations

from collections.abc import Callable

from control_msgs.msg import DynamicJointState
from lucy_config_generator.motors import motor_values_by_joint
import yaml


def extract_voltages(msg: DynamicJointState) -> dict[str, float]:
    return {
        joint: value
        for joint, interfaces in zip(msg.joint_names, msg.interface_values)
        for name, value in zip(interfaces.interface_names, interfaces.values)
        if name == "voltage"
    }


def load_min_volts(
    read_active_config: Callable[[], str],
) -> tuple[dict[str, float], str]:
    try:
        config = yaml.safe_load(read_active_config()) or {}
        return motor_values_by_joint(config, "min_volts"), ""
    except (OSError, KeyError, yaml.YAMLError) as e:
        return {}, str(e)

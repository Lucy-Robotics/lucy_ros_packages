# Copyright 2026 Lucy Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

"""Project one board's plugs into the firmware spec the RP2040 build consumes."""

from __future__ import annotations

from typing import Any

from lucy_config_generator.plug import HeaderPlug
from lucy_config_generator.plug import resolve_plug
from lucy_config_generator.schema import assign_virtual_pins
from lucy_config_generator.schema import derive_ros2_node_name
from lucy_config_generator.servo_profile import amplitude_rad
from lucy_config_generator.servo_profile import MAX_PULSE_US
from lucy_config_generator.servo_profile import MIN_PULSE_US

import yaml


def project_firmware_spec(data: dict[str, Any], board_id: str) -> dict[str, Any]:
    """
    Return the per-board firmware spec for enabled actuators on ``board_id``.

    :param data: Validated hardware YAML.
    :param board_id: Board whose actuators are projected.
    """
    assign_virtual_pins(data)
    board = data['boards'][board_id]
    pwm: list[dict[str, Any]] = []
    i2c_pwm: list[dict[str, Any]] = []
    devices = {
        str(item['id']): item
        for item in board.get('i2c_devices') or []
        if isinstance(item, dict) and item.get('id')
    }
    for actuator in data['actuators']:
        if actuator.get('board') != board_id or not actuator.get('enabled'):
            continue
        plug = resolve_plug(actuator, board)
        row = _pulse_fields(actuator)
        row['virtual_pin'] = int(actuator['virtual_pin'])
        if isinstance(plug, HeaderPlug):
            row['header'] = plug.header
            row['identity'] = f'Servo{plug.header}'
            pwm.append(row)
        else:
            device = devices[plug.device]
            address = int(device['address'])
            row['bus'] = 0
            row['address'] = address
            row['channel'] = plug.channel
            row['identity'] = f'I2C0:PCA9685:{address}:{plug.channel}'
            i2c_pwm.append(row)
    pwm.sort(key=lambda item: item['virtual_pin'])
    i2c_pwm.sort(key=lambda item: item['virtual_pin'])
    return {
        'board_id': board_id,
        'serial_id': board.get('serial_id') or '',
        'node_name': derive_ros2_node_name(board_id),
        'pwm': pwm,
        'i2c_pwm': i2c_pwm,
    }


def render_firmware_spec(data: dict[str, Any], board_id: str) -> str:
    """YAML text of :func:`project_firmware_spec`."""
    return yaml.safe_dump(
        project_firmware_spec(data, board_id),
        sort_keys=False,
        default_flow_style=False,
    )


def _pulse_fields(actuator: dict[str, Any]) -> dict[str, Any]:
    return {
        'amplitude_rad': amplitude_rad(str(actuator['servo_type'])),
        'min_pulse_us': MIN_PULSE_US,
        'max_pulse_us': MAX_PULSE_US,
    }

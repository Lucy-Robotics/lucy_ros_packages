# Copyright 2026 Lucy Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

"""Enabled header plugs become pwm rows. Disabled actuators stay out of the spec."""

from __future__ import annotations

from pathlib import Path

from lucy_config_generator.firmware_spec import project_firmware_spec
from lucy_config_generator.servo_profile import amplitude_rad
import pytest
import yaml

_FIXTURES = Path(__file__).resolve().parent / 'fixtures'


def _mapping() -> dict:
    with (_FIXTURES / 'test_mapping.yaml').open(encoding='utf-8') as handle:
        return yaml.safe_load(handle)


def test_enabled_header_becomes_one_pwm_row():
    spec = project_firmware_spec(_mapping(), 'rp2040_left_arm')
    assert spec['node_name'] == 'left_arm'
    assert spec['i2c_pwm'] == []
    assert spec['pwm'] == [
        {
            'amplitude_rad': amplitude_rad('270'),
            'min_pulse_us': 500,
            'max_pulse_us': 2500,
            'virtual_pin': 0,
            'header': 10,
            'identity': 'Servo10',
        }
    ]


def test_pca_row_uses_the_same_projector():
    data = _mapping()
    data['boards']['rp2040_left_arm']['i2c_devices'] = [
        {'id': 'pca0', 'type': 'pca9685', 'address': 64, 'channels': 16}
    ]
    data['actuators'][0].pop('physical_pin')
    data['actuators'][0]['plug'] = {'kind': 'pca9685', 'device': 'pca0', 'channel': 5}
    data['actuators'][1]['enabled'] = True
    spec = project_firmware_spec(data, 'rp2040_left_arm')
    assert spec['i2c_pwm'][0]['identity'] == 'I2C0:PCA9685:64:5'
    assert spec['i2c_pwm'][0]['channel'] == 5
    assert spec['pwm'][0]['identity'] == 'Servo11'


def test_disabled_actuators_are_omitted():
    data = _mapping()
    data['actuators'][0]['enabled'] = False
    spec = project_firmware_spec(data, 'rp2040_left_arm')
    assert spec['pwm'] == []


@pytest.mark.parametrize(
    ('servo_type', 'expected'),
    [('180', 3.141592653589793), ('270', 4.71238898038469), ('300', 5.235987755982989)],
)
def test_servo_type_amplitudes(servo_type: str, expected: float):
    assert amplitude_rad(servo_type) == pytest.approx(expected)

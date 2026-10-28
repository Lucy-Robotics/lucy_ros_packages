# Copyright 2026 Lucy Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

"""Header plug resolution. Legacy rows keep physical_pin as the Servo header."""

from __future__ import annotations

from copy import deepcopy
from pathlib import Path

from lucy_config_generator.plug import HeaderPlug
from lucy_config_generator.plug import PcaPlug
from lucy_config_generator.plug import assign_pca_plug
from lucy_config_generator.plug import assign_plug
from lucy_config_generator.plug import resolve_plug
from lucy_config_generator.schema import validate_hardware_yaml
import pytest
import yaml

_FIXTURES = Path(__file__).resolve().parent / 'fixtures'
_THAIS = (
    Path(__file__).resolve().parents[3]
    / 'thais_urdf'
    / 'config'
    / 'hardware'
    / 'configs'
    / 'default.yaml'
)
_INMOOV = (
    Path(__file__).resolve().parents[3]
    / 'inmoov_urdf'
    / 'config'
    / 'hardware'
    / 'configs'
    / 'default.yaml'
)


def _mapping() -> dict:
    with (_FIXTURES / 'test_mapping.yaml').open(encoding='utf-8') as handle:
        return yaml.safe_load(handle)


def test_legacy_physical_pin_is_the_header():
    data = _mapping()
    actuator = data['actuators'][0]
    assert 'plug' not in actuator
    assert resolve_plug(actuator).header == actuator['physical_pin']
    validate_hardware_yaml(data)


def test_assign_plug_keeps_physical_pin_equal_to_header():
    actuator = {'id': 'jaw'}
    assign_plug(actuator, 12)
    assert actuator['plug'] == {'kind': 'servo_header', 'header': 12}
    assert actuator['physical_pin'] == 12
    resolved = resolve_plug(actuator)
    assert isinstance(resolved, HeaderPlug)
    assert resolved.header == 12


def test_assign_pca_plug_drops_physical_pin():
    actuator = {'id': 'jaw', 'physical_pin': 10}
    assign_pca_plug(actuator, 'pca0', 3)
    assert 'physical_pin' not in actuator
    resolved = resolve_plug(
        actuator,
        {'i2c_devices': [{'id': 'pca0', 'type': 'pca9685', 'address': 64, 'channels': 16}]},
    )
    assert isinstance(resolved, PcaPlug)
    assert resolved.device == 'pca0'
    assert resolved.channel == 3


def test_board_class_is_derived_from_i2c_devices():
    data = _mapping()
    data['boards']['rp2040_left_arm']['board_class'] = 'internal_servo_only'
    data['boards']['rp2040_left_arm']['i2c_devices'] = [
        {'id': 'pca0', 'type': 'pca9685', 'address': 64, 'channels': 16}
    ]
    validate_hardware_yaml(data)
    assert data['boards']['rp2040_left_arm']['board_class'] == 'internal_servo_i2c_pwm'
    data['boards']['rp2040_left_arm']['i2c_devices'] = []
    validate_hardware_yaml(data)
    assert data['boards']['rp2040_left_arm']['board_class'] == 'internal_servo_only'


def test_mismatched_plug_and_physical_pin_is_rejected():
    data = _mapping()
    data['actuators'][0]['plug'] = {'kind': 'servo_header', 'header': 4}
    with pytest.raises(ValueError, match='physical_pin must equal plug.header'):
        validate_hardware_yaml(data)


def test_duplicate_header_is_rejected():
    data = _mapping()
    data['actuators'][1]['physical_pin'] = data['actuators'][0]['physical_pin']
    with pytest.raises(ValueError, match='duplicate header'):
        validate_hardware_yaml(data)


@pytest.mark.parametrize('headers', [(1, 17), (2, 18)])
def test_shared_pwm_pair_is_rejected(headers):
    data = _mapping()
    left, right = headers
    data['actuators'][0]['physical_pin'] = left
    data['actuators'][1]['physical_pin'] = right
    with pytest.raises(ValueError, match='share a PWM slice'):
        validate_hardware_yaml(data)


def test_header_above_18_is_rejected():
    data = _mapping()
    board = data['boards']['rp2040_left_arm']
    board['internal_servo_slots'] = 20
    data['actuators'][0]['physical_pin'] = 19
    with pytest.raises(ValueError, match='out of range 1..18'):
        validate_hardware_yaml(data)


def _attach_pca(data: dict, address: int = 64, device: str = 'pca0') -> None:
    data['boards']['rp2040_left_arm']['i2c_devices'] = [
        {'id': device, 'type': 'pca9685', 'address': address, 'channels': 16}
    ]


def test_pca_plug_does_not_occupy_a_header():
    data = _mapping()
    _attach_pca(data)
    actuator = data['actuators'][0]
    actuator.pop('physical_pin')
    actuator['plug'] = {'kind': 'pca9685', 'device': 'pca0', 'channel': 5}
    validate_hardware_yaml(data)
    assert 'physical_pin' not in data['actuators'][0]
    assert data['actuators'][1]['physical_pin'] == 11


def test_same_pca_address_twice_is_rejected():
    data = _mapping()
    data['boards']['rp2040_left_arm']['i2c_devices'] = [
        {'id': 'pca0', 'type': 'pca9685', 'address': 64, 'channels': 16},
        {'id': 'pca1', 'type': 'pca9685', 'address': 64, 'channels': 16},
    ]
    with pytest.raises(ValueError, match='duplicate i2c address'):
        validate_hardware_yaml(data)


def test_two_pca_addresses_on_one_board_are_valid():
    data = _mapping()
    data['boards']['rp2040_left_arm']['i2c_devices'] = [
        {'id': 'pca0', 'type': 'pca9685', 'address': 0x40, 'channels': 16},
        {'id': 'pca1', 'type': 'pca9685', 'address': 0x41, 'channels': 16},
    ]
    validate_hardware_yaml(data)


def test_pca_device_must_be_on_the_actuator_board():
    data = _mapping()
    data['actuators'][0].pop('physical_pin')
    data['actuators'][0]['plug'] = {'kind': 'pca9685', 'device': 'missing', 'channel': 0}
    with pytest.raises(ValueError, match='not on this board'):
        validate_hardware_yaml(data)


def test_unknown_plug_kind_is_rejected():
    data = _mapping()
    data['actuators'][0]['plug'] = {'kind': 'uart', 'header': 1}
    with pytest.raises(ValueError, match='unsupported plug kind'):
        validate_hardware_yaml(data)


@pytest.mark.parametrize('path', [_THAIS, _INMOOV])
def test_shipped_robot_yaml_still_validates(path: Path):
    if not path.is_file():
        pytest.skip(f'missing {path}')
    with path.open(encoding='utf-8') as handle:
        data = yaml.safe_load(handle)
    validate_hardware_yaml(deepcopy(data))

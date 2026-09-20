# Copyright 2025 Sentience Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

"""Unit tests for Rust firmware YAML generation."""

from __future__ import annotations

import math

from lucy_config_generator.generate import generate_from_xacro_string_for_tests
from lucy_config_generator.generate import render_firmware_yaml


def _rad(deg: float) -> float:
    return math.radians(deg)


MINIMAL = {
    'version': 1,
    'robot_name': 'test',
    'firmware': {'source_dir': 'lucy_embedded_firmware', 'build_dir': 'build/firmware'},
    'controller_manager': {'update_rate': 100},
    'boards': {
        'rp2040_left_arm': {
            'serial_id': 'ABC',
            'board_class': 'internal_servo_only',
            'slave_address': 1,
            'internal_servo_slots': 18,
            'firmware_target': 'lucy_left_arm',
            'compile_definition': 'USE_LEFT',
            'topic_actuators': 'actuators/left_arm',
            'topic_sensors': 'sensors/left_arm',
            'controller': {
                'name': 'left_arm_controller',
                'type': 'joint_trajectory_controller/JointTrajectoryController',
            },
        },
        'rp2040_torso_head': {
            'serial_id': '',
            'board_class': 'internal_servo_i2c_pwm',
            'slave_address': 1,
            'internal_servo_slots': 18,
            'firmware_target': 'lucy_torso',
            'compile_definition': 'USE_TORSO',
            'topic_actuators': 'actuators/torso',
            'topic_sensors': 'sensors/torso',
            'controller': {
                'name': 'torso_controller',
                'type': 'joint_trajectory_controller/JointTrajectoryController',
            },
        },
        'rp2040_so_arm': {
            'serial_id': '',
            'board_class': 'bus_servo_only',
            'slave_address': 2,
            'internal_servo_slots': 18,
            'firmware_target': 'so_arm',
            'compile_definition': 'USE_SO',
            'topic_actuators': 'actuators/so_arm',
            'topic_sensors': 'sensors/so_arm',
            'controller': {
                'name': 'so_arm_controller',
                'type': 'joint_trajectory_controller/JointTrajectoryController',
            },
        },
    },
    'actuators': [
        {
            'id': 'left_elbow',
            'urdf_joint': 'left_elbow_joint',
            'board': 'rp2040_left_arm',
            'virtual_pin': 0,
            'physical_pin': 10,
            'servo_type': '270',
            'offset_rad': 0.0,
            'direction': 1,
            'scale': 1,
            'servo_min_rad': 0.0,
            'servo_max_rad': _rad(270),
            'servo_default_rad': _rad(135),
            'enabled': True,
        },
        {
            'id': 'head_jaw',
            'urdf_joint': 'jaw_joint',
            'board': 'rp2040_torso_head',
            'virtual_pin': 0,
            'physical_pin': 12,
            'servo_type': '180',
            'offset_rad': 0.0,
            'direction': -1,
            'scale': 1,
            'servo_min_rad': 0.0,
            'servo_max_rad': _rad(180),
            'servo_default_rad': _rad(90),
            'enabled': True,
        },
        {
            'id': 'disabled_joint',
            'urdf_joint': 'disabled_joint',
            'board': 'rp2040_left_arm',
            'virtual_pin': 1,
            'physical_pin': 11,
            'servo_type': '300',
            'offset_rad': 0.0,
            'direction': 1,
            'scale': 0.25,
            'servo_min_rad': 0.0,
            'servo_max_rad': _rad(300),
            'servo_default_rad': _rad(150),
            'enabled': False,
        },
        {
            'id': 'so_joint',
            'urdf_joint': 'so_joint',
            'board': 'rp2040_so_arm',
            'virtual_pin': 0,
            'physical_pin': 1,
            'servo_type': '180',
            'offset_rad': 0.0,
            'direction': 1,
            'scale': 1,
            'servo_min_rad': 0.0,
            'servo_max_rad': _rad(180),
            'servo_default_rad': _rad(90),
            'enabled': True,
        },
    ],
    'sensors': [
        {
            'id': 'left_gripper_pressure',
            'type': 'pressure',
            'associated_actuator': 'left_elbow',
            'board': 'rp2040_left_arm',
            'virtual_pin': 0,
            'physical_pin': 1,
            'min_value': 0,
            'max_value': 4095,
            'enabled': True,
        },
    ],
}


def test_render_firmware_yaml_pwm_board():
    text = render_firmware_yaml(MINIMAL, 'rp2040_left_arm')
    assert 'board_id: rp2040_left_arm' in text
    assert 'board_class: internal_servo_only' in text
    assert 'firmware_crate: firmwares/rp2040_internal_pwm' in text
    assert 'left_elbow' in text
    assert 'channel: Servo10' in text
    assert 'driver: PwmServoDriver' in text
    assert 'disabled_joint' not in text
    assert 'left_gripper_pressure' in text
    assert 'driver: PressureSensorDriver' in text
    assert 'channel: ADC0' in text
    assert f'max_angle: {_rad(270)}' in text


def test_render_firmware_yaml_includes_serial_id():
    text = render_firmware_yaml(MINIMAL, 'rp2040_left_arm')
    assert 'serial_id: "ABC"' in text


def test_render_firmware_yaml_i2c_board():
    text = render_firmware_yaml(MINIMAL, 'rp2040_torso_head')
    assert 'board_class: internal_servo_i2c_pwm' in text
    assert 'firmware_crate: firmwares/rp2040_i2c_pwm' in text
    assert 'channel: Servo12' in text
    assert 'head_jaw' in text


def test_render_firmware_yaml_bus_servo_board():
    text = render_firmware_yaml(MINIMAL, 'rp2040_so_arm')
    assert 'board_class: bus_servo_only' in text
    assert 'firmware_crate: firmwares/rp2040_bus_servo' in text
    assert 'driver: BusServoDriver' in text
    assert 'channel: UART0:1' in text
    assert 'slave_address: 2' in text


def test_generate_emits_yaml_not_c():
    urdf = """<?xml version="1.0"?>
<robot name="t">
  <joint name="left_elbow_joint" type="revolute">
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <joint name="jaw_joint" type="revolute">
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <joint name="disabled_joint" type="revolute">
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <joint name="so_joint" type="revolute">
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
</robot>
"""
    out = generate_from_xacro_string_for_tests(
        MINIMAL, urdf, targets={'firmware'}, boards_filter={'rp2040_left_arm'}
    )
    assert 'config_rp2040_left_arm.yaml' in out
    assert 'config_rp2040_left_arm.c' not in out

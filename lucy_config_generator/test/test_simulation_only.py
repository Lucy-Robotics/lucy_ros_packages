# Copyright 2025 Sentience Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

"""Simulation-only generation keeps the same 3 controllers and 1 ros2_control block per board."""

from __future__ import annotations

from pathlib import Path

from lucy_config_generator.generate import generate_from_xacro_string_for_tests

import yaml

_FIXTURES = Path(__file__).resolve().parent / 'fixtures'


def _load_mapping() -> dict:
    with (_FIXTURES / 'test_mapping.yaml').open(encoding='utf-8') as f:
        return yaml.safe_load(f)


def _fixture_urdf_xml() -> str:
    return (_FIXTURES / 'test_robot.urdf.xacro').read_text(encoding='utf-8')


def test_simulation_only_keeps_three_controllers_and_per_board_blocks():
    data = _load_mapping()
    out = generate_from_xacro_string_for_tests(
        data,
        _fixture_urdf_xml(),
        targets={'ros2_control', 'controllers'},
        simulation_only=True,
    )
    assert not any(k.startswith('config_') for k in out), \
        'simulation_only must not produce firmware'

    xacro = out['inmoov_ros2_control.xacro']
    expected_boards = list(data['boards'].keys())
    assert xacro.count('<ros2_control name="') == len(expected_boards)
    assert 'use_mock_hardware' in xacro
    # Real and mock hardware now share the LucySystemHardware plugin so URDF
    # limit clamping is exercised on both paths; mock differs only in that
    # ``publish_actuators=false`` suppresses the optional JointState debug publisher.
    assert 'lucy_ros2_control/LucySystemHardware' in xacro
    assert 'publish_actuators' in xacro
    assert 'mock_components/GenericSystem' not in xacro

    actuator_joints = sorted(a['urdf_joint'] for a in data['actuators'])
    for joint in actuator_joints:
        assert f'<joint name="{joint}"' in xacro

    ctrl_yaml = yaml.safe_load(out['controllers.yaml'])
    cm = ctrl_yaml['controller_manager']['ros__parameters']
    expected_controllers = [
        data['boards'][b]['controller']['name'] for b in expected_boards
    ]
    for name in expected_controllers:
        assert name in cm, f'controller {name!r} missing from controller_manager params'
    assert 'joint_state_broadcaster' in cm
    assert 'lucy_sim_controller' not in cm

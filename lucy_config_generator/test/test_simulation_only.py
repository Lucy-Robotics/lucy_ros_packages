# Copyright 2026 Lucy Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

"""Simulation-only generation keeps per-board hardware blocks and trajectory controllers."""

from __future__ import annotations

from pathlib import Path

from lucy_config_generator.generate import generate_from_xacro_string_for_tests
from lucy_config_generator.generate import torque_enable_controller_name

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
    # ``publish_actuators=false`` suppresses the micro-ROS actuator publisher.
    assert 'lucy_ros2_control/LucySystemHardware' in xacro
    assert 'publish_actuators' in xacro
    assert 'mock_components/GenericSystem' not in xacro

    actuator_joints = sorted(a['urdf_joint'] for a in data['actuators'])
    for joint in actuator_joints:
        assert f'<joint name="{joint}"' in xacro
    assert '<command_interface name="torque_enable"/>' in xacro

    ctrl_yaml = yaml.safe_load(out['controllers.yaml'])
    cm = ctrl_yaml['controller_manager']['ros__parameters']
    expected_controllers = [
        data['boards'][b]['controller']['name'] for b in expected_boards
    ]
    for name in expected_controllers:
        assert name in cm, f'controller {name!r} missing from controller_manager params'
        torque = torque_enable_controller_name(name)
        assert cm[torque]['type'] == 'forward_command_controller/ForwardCommandController'
        torque_params = ctrl_yaml[torque]['ros__parameters']
        traj_params = ctrl_yaml[name]['ros__parameters']
        assert torque_params['interface_name'] == 'torque_enable'
        assert torque_params['joints'] == traj_params['joints']
    assert 'joint_state_broadcaster' in cm
    assert 'lucy_sim_controller' not in cm

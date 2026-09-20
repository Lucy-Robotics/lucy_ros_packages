# Copyright 2025 Sentience Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

"""Golden tests for lucy_config_generator (#96)."""

from __future__ import annotations

from pathlib import Path

from lucy_config_generator.generate import generate_from_xacro_string_for_tests
from lucy_config_generator.schema import derive_ros2_hardware_name
from lucy_config_generator.schema import derive_ros2_node_name
from lucy_config_generator.schema import GENERATED_FILES_DEFAULTS
from lucy_config_generator.schema import resolve_generated_files
from lucy_config_generator.schema import validate_hardware_yaml

import pytest
import yaml

_FIXTURES = Path(__file__).resolve().parent / 'fixtures'


def _fixture_urdf_xml() -> str:
    """Return mock robot XML from test_robot.urdf.xacro for joint-name extraction."""
    return (_FIXTURES / 'test_robot.urdf.xacro').read_text(encoding='utf-8')


def _load_mapping() -> dict:
    with (_FIXTURES / 'test_mapping.yaml').open(encoding='utf-8') as f:
        return yaml.safe_load(f)


def test_schema_accepts_fixture():
    validate_hardware_yaml(_load_mapping())


def test_ros2_names_derived_from_board_id():
    assert derive_ros2_hardware_name('rp2040_left_arm') == 'LucyHardwareLeftArm'
    assert derive_ros2_node_name('rp2040_left_arm') == 'left_arm'
    assert derive_ros2_hardware_name('rp2040_torso_head') == 'LucyHardwareTorsoHead'
    assert derive_ros2_node_name('rp2040_torso_head') == 'torso_head'


def test_schema_rejects_bad_version():
    data = _load_mapping()
    data['version'] = 2
    with pytest.raises(ValueError, match='version'):
        validate_hardware_yaml(data)


def test_schema_rejects_empty_compile_definition():
    data = _load_mapping()
    data['boards']['rp2040_left_arm']['compile_definition'] = ''
    with pytest.raises(ValueError, match='compile_definition'):
        validate_hardware_yaml(data)


def test_schema_rejects_empty_controller_name():
    data = _load_mapping()
    data['boards']['rp2040_left_arm']['controller']['name'] = ''
    with pytest.raises(ValueError, match=r'controller\.name'):
        validate_hardware_yaml(data)


def test_schema_rejects_invalid_internal_servo_slots():
    data = _load_mapping()
    data['boards']['rp2040_left_arm']['internal_servo_slots'] = 0
    with pytest.raises(ValueError, match='internal_servo_slots must be >= 1'):
        validate_hardware_yaml(data)


def test_schema_rejects_null_virtual_pin():
    data = _load_mapping()
    data['actuators'][0]['virtual_pin'] = None
    with pytest.raises(ValueError, match='virtual_pin must be an integer'):
        validate_hardware_yaml(data)


def test_schema_rejects_servo_min_greater_than_max():
    data = _load_mapping()
    data['actuators'][0]['servo_min_rad'] = 10
    data['actuators'][0]['servo_max_rad'] = 5
    data['actuators'][0]['servo_default_rad'] = 7
    with pytest.raises(ValueError, match='servo_min_rad .* must be <= servo_max_rad'):
        validate_hardware_yaml(data)


def test_schema_rejects_physical_pin_above_board_slot_limit():
    data = _load_mapping()
    data['boards']['rp2040_left_arm']['internal_servo_slots'] = 8
    data['actuators'][0]['physical_pin'] = 9
    with pytest.raises(ValueError, match=r'physical_pin 9 out of range 1..8'):
        validate_hardware_yaml(data)


def test_schema_rejects_non_bool_enabled_actuator():
    data = _load_mapping()
    data['actuators'][0]['enabled'] = 'fal'
    with pytest.raises(ValueError, match='enabled must be a boolean'):
        validate_hardware_yaml(data)


def test_schema_rejects_non_bool_enabled_sensor():
    data = _load_mapping()
    data['sensors'][0]['enabled'] = 'fal'
    with pytest.raises(ValueError, match='enabled must be a boolean'):
        validate_hardware_yaml(data)


def test_schema_rejects_non_numeric_offset_rad():
    data = _load_mapping()
    data['actuators'][0]['offset_rad'] = None
    with pytest.raises(ValueError, match='offset_rad must be numeric'):
        validate_hardware_yaml(data)


def test_schema_rejects_non_unit_direction():
    data = _load_mapping()
    data['actuators'][0]['direction'] = 0
    with pytest.raises(ValueError, match='direction must be -1 or 1'):
        validate_hardware_yaml(data)


def test_schema_rejects_zero_scale():
    data = _load_mapping()
    data['actuators'][0]['scale'] = 0
    with pytest.raises(ValueError, match='scale must be non-zero'):
        validate_hardware_yaml(data)


def test_schema_rejects_empty_sensor_id():
    data = _load_mapping()
    data['sensors'][0]['id'] = ''
    with pytest.raises(ValueError, match=r'sensor .*: id must be a non-empty string'):
        validate_hardware_yaml(data)


def test_schema_rejects_empty_sensor_type():
    data = _load_mapping()
    data['sensors'][0]['type'] = ''
    with pytest.raises(ValueError, match=r'sensor .*: type must be a non-empty string'):
        validate_hardware_yaml(data)


def test_schema_rejects_sensor_min_value_greater_than_max_value():
    data = _load_mapping()
    data['sensors'][0]['min_value'] = 2000
    data['sensors'][0]['max_value'] = 0
    with pytest.raises(ValueError, match=r'sensor .*: min_value .* must be <= max_value'):
        validate_hardware_yaml(data)


def test_schema_reports_missing_virtual_pin_when_gap_exists():
    data = _load_mapping()
    data['actuators'][2]['virtual_pin'] = 3
    with pytest.raises(ValueError, match=r'board .*: missing virtual_pin indices \[[0-9, ]+\]'):
        validate_hardware_yaml(data)


def test_schema_suppresses_sensor_contiguity_when_sensor_has_item_error():
    data = _load_mapping()
    data['sensors'][0]['associated_actuator'] = 'does_not_exist'
    with pytest.raises(ValueError) as exc:
        validate_hardware_yaml(data)
    msg = str(exc.value)
    assert 'associated_actuator does_not_exist not found' in msg
    assert 'sensor virtual_pin must be contiguous' not in msg
    assert 'missing sensor virtual_pin indices' not in msg


def test_golden_firmware_left_arm():
    data = _load_mapping()
    got = generate_from_xacro_string_for_tests(data, _fixture_urdf_xml(), {'firmware'}, None)[
        'config_rp2040_left_arm.yaml'
    ]
    assert 'board_id: rp2040_left_arm' in got
    assert 'driver: PwmServoDriver' in got
    assert 'channel: Servo10' in got
    assert 'hardware:' in got
    assert 'firmware_crate: firmwares/rp2040_servo2040' in got


def test_golden_firmware_right_arm():
    data = _load_mapping()
    got = generate_from_xacro_string_for_tests(data, _fixture_urdf_xml(), {'firmware'}, None)[
        'config_rp2040_right_arm.yaml'
    ]
    assert 'board_id: rp2040_right_arm' in got
    assert 'driver: PwmServoDriver' in got
    assert 'min_angle:' in got


def test_golden_firmware_torso():
    data = _load_mapping()
    got = generate_from_xacro_string_for_tests(data, _fixture_urdf_xml(), {'firmware'}, None)[
        'config_rp2040_torso_head.yaml'
    ]
    assert 'board_id: rp2040_torso_head' in got
    assert 'min_pulse:' in got


def test_golden_ros2_control():
    data = _load_mapping()
    got = generate_from_xacro_string_for_tests(data, _fixture_urdf_xml(), {'ros2_control'}, None)[
        GENERATED_FILES_DEFAULTS['ros2_control_xacro']
    ]
    expected = (_FIXTURES / 'golden_inmoov_ros2_control.xacro').read_text(encoding='utf-8')
    assert got == expected


def test_ros2_control_emits_urdf_limits_on_command_interface():
    data = _load_mapping()
    got = generate_from_xacro_string_for_tests(data, _fixture_urdf_xml(), {'ros2_control'}, None)[
        GENERATED_FILES_DEFAULTS['ros2_control_xacro']
    ]
    assert 'name="left_shoulder_y_link_joint"' in got
    assert '<param name="min">0.0</param>' in got
    assert '<param name="max">1.0</param>' in got
    # Every actuated revolute in the fixture has limits 0..1 rad.
    assert got.count('<param name="min">') == 5
    assert got.count('<param name="max">') == 5


def test_golden_controllers_extra_joints():
    data = _load_mapping()
    got = generate_from_xacro_string_for_tests(data, _fixture_urdf_xml(), {'controllers'}, None)[
        GENERATED_FILES_DEFAULTS['controllers_yaml']
    ]
    expected = (_FIXTURES / 'golden_controllers.yaml').read_text(encoding='utf-8')
    assert got == expected


def test_mimic_joints_excluded_from_extra_joints():
    """A <mimic> joint is derived by robot_state_publisher, not broadcast at 0.0."""
    data = _load_mapping()
    mimic_joint = (
        '  <joint name="passive_mimic_joint" type="revolute">\n'
        '    <parent link="a"/>\n'
        '    <child link="b"/>\n'
        '    <axis xyz="0 0 1"/>\n'
        '    <limit effort="1" lower="0" upper="1" velocity="1"/>\n'
        '    <mimic joint="left_a_joint" multiplier="1.0" offset="0.0"/>\n'
        '  </joint>\n'
    )
    urdf = _fixture_urdf_xml().replace('</robot>', mimic_joint + '</robot>')
    got = generate_from_xacro_string_for_tests(data, urdf, {'controllers'}, None)[
        GENERATED_FILES_DEFAULTS['controllers_yaml']
    ]
    assert 'passive_mimic_joint' not in got
    assert '- passive_extra_joint' in got


def test_boards_filter_emits_subset():
    data = _load_mapping()
    got = generate_from_xacro_string_for_tests(
        data,
        _fixture_urdf_xml(),
        {'firmware'},
        {'rp2040_left_arm'},
    )
    assert set(got.keys()) == {'config_rp2040_left_arm.yaml'}


def test_generated_files_defaults_when_section_absent():
    assert resolve_generated_files(_load_mapping()) == GENERATED_FILES_DEFAULTS


def test_generated_files_override_basenames():
    data = _load_mapping()
    data['generated_files'] = {
        'ros2_control_xacro': 'thais_ros2_control.xacro',
        'controllers_yaml': 'thais_controllers.yaml',
    }
    assert resolve_generated_files(data) == {
        'ros2_control_xacro': 'thais_ros2_control.xacro',
        'controllers_yaml': 'thais_controllers.yaml',
    }


def test_generated_files_rejects_path_separator():
    data = _load_mapping()
    data['generated_files'] = {'controllers_yaml': 'config/controllers.yaml'}
    with pytest.raises(ValueError, match='bare filename'):
        validate_hardware_yaml(data)


def test_generated_files_basenames_drive_output_keys():
    data = _load_mapping()
    data['generated_files'] = {
        'ros2_control_xacro': 'thais_ros2_control.xacro',
        'controllers_yaml': 'thais_controllers.yaml',
    }
    got = generate_from_xacro_string_for_tests(
        data, _fixture_urdf_xml(), {'ros2_control', 'controllers'}, None
    )
    assert 'thais_ros2_control.xacro' in got
    assert 'thais_controllers.yaml' in got
    assert GENERATED_FILES_DEFAULTS['ros2_control_xacro'] not in got


def _bus_servo_mapping() -> dict:
    """Fixture mapping with its first board switched to smart bus servos."""
    data = _load_mapping()
    board_id = next(iter(data['boards']))
    data['boards'][board_id]['board_class'] = 'bus_servo_only'
    return data, board_id


def test_schema_accepts_bus_servo_board_class():
    data, _ = _bus_servo_mapping()
    validate_hardware_yaml(data)


def test_bus_servo_board_emits_type_and_bus_id():
    """A bus joint is addressed by id on a shared UART, not by a board pin."""
    data, board_id = _bus_servo_mapping()
    out = generate_from_xacro_string_for_tests(
        data, _fixture_urdf_xml(), targets={'ros2_control'}
    )
    xacro = out[resolve_generated_files(data)['ros2_control_xacro']]
    assert '<param name="type">bus_servo</param>' in xacro

    actuators = [a for a in data['actuators'] if a['board'] == board_id]
    assert actuators, 'fixture board has no actuators'
    for a in actuators:
        assert f'<param name="bus_id">{a["physical_pin"]}</param>' in xacro


def test_pwm_board_emits_neither_type_nor_bus_id():
    """The plugin defaults joints to pwm_servo, so PWM output must not change."""
    data = _load_mapping()
    out = generate_from_xacro_string_for_tests(
        data, _fixture_urdf_xml(), targets={'ros2_control'}
    )
    xacro = out[resolve_generated_files(data)['ros2_control_xacro']]
    assert '<param name="type">' not in xacro
    assert '<param name="bus_id">' not in xacro


def test_bus_servo_board_generates_no_firmware_c():
    """Bus boards run the generic Rust firmware; there is no per-board C."""
    data, board_id = _bus_servo_mapping()
    out = generate_from_xacro_string_for_tests(
        data, _fixture_urdf_xml(), targets={'firmware'}
    )
    assert f'config_{board_id}.c' not in out


def test_bus_servo_board_generates_rust_layout():
    data, board_id = _bus_servo_mapping()
    out = generate_from_xacro_string_for_tests(
        data, _fixture_urdf_xml(), targets={'firmware'}
    )
    assert f'config_{board_id}.c' not in out
    yaml_text = out[f'config_{board_id}.yaml']
    assert 'board_class: bus_servo_only' in yaml_text
    assert 'firmware_crate: firmwares/rp2040_bus_servo' in yaml_text
    assert 'driver: BusServoDriver' in yaml_text
    assert 'channel: UART0:' in yaml_text


def test_slot_count_is_the_board_joint_count():
    """Enabled actuators appear in architecture YAML; disabled are filtered."""
    data, board_id = _bus_servo_mapping()
    out = generate_from_xacro_string_for_tests(
        data, _fixture_urdf_xml(), targets={'firmware'}
    )
    yaml_text = out[f'config_{board_id}.yaml']
    enabled = [
        a for a in data['actuators'] if a['board'] == board_id and a.get('enabled', True)
    ]
    for a in enabled:
        assert a['id'] in yaml_text

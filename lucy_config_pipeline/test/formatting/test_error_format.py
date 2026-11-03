import json

from src.error_format import format_error_line


def test_board_error_formats_to_json_field_path():
    s = format_error_line(
        'board rp2040_left_arm: controller.name must be a non-empty string'
    )
    o = json.loads(s)
    assert o['field'] == 'boards.rp2040_left_arm.controller.name'
    assert o['field_path'] == ['boards', 'rp2040_left_arm', 'controller', 'name']
    assert 'non-empty' in o['message']


def test_missing_root_key_formats_to_json():
    s = format_error_line('missing root key: boards')
    o = json.loads(s)
    assert o['field'] == 'boards'
    assert o['field_path'] == ['boards']
    assert 'missing root key' in o['message']


def test_yaml_parser_line_column_are_exposed():
    s = format_error_line('in "<unicode string>", line 67, column 18:')
    o = json.loads(s)
    assert o['field'] == 'yaml.syntax'
    assert o['field_path'] == ['yaml', 'syntax']
    assert o['line'] == 67
    assert o['column'] == 18


def test_board_duplicate_sensor_physical_pin_message_is_mapped():
    s = format_error_line('board rp2040_left_arm: duplicate sensor physical_pin')
    o = json.loads(s)
    assert o['field'] == 'boards.rp2040_left_arm.duplicate'
    assert o['field_path'] == ['boards', 'rp2040_left_arm', 'duplicate']
    assert o['message'] == 'duplicate sensor physical_pin'

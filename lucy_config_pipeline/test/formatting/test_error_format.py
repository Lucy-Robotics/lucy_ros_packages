import json

from src.error_format import format_error_line


def test_board_error_formats_to_json_field_path():
    s = format_error_line(
        "board rp2040_left_arm: controller.name must be a non-empty string"
    )
    o = json.loads(s)
    assert o["field"] == "boards.rp2040_left_arm.controller.name"
    assert o["field_path"] == ["boards", "rp2040_left_arm", "controller", "name"]
    assert "non-empty" in o["message"]


def test_missing_root_key_formats_to_json():
    s = format_error_line("missing root key: boards")
    o = json.loads(s)
    assert o["field"] == "boards"
    assert o["field_path"] == ["boards"]
    assert "missing root key" in o["message"]


def test_yaml_parser_line_column_are_exposed():
    s = format_error_line('in "<unicode string>", line 67, column 18:')
    o = json.loads(s)
    assert o["field"] == "yaml.syntax"
    assert o["field_path"] == ["yaml", "syntax"]
    assert o["line"] == 67
    assert o["column"] == 18


def test_board_virtual_pin_gap_message_is_preserved():
    s = format_error_line(
        "board rp2040_left_arm: virtual_pin must be contiguous from 0..N-1, got [0, 1, 3]"
    )
    o = json.loads(s)
    assert o["field"] == "boards.rp2040_left_arm.virtual_pin"
    assert o["field_path"] == ["boards", "rp2040_left_arm", "virtual_pin"]
    assert o["message"] == "virtual_pin must be contiguous from 0..N-1, got [0, 1, 3]"


def test_board_missing_virtual_pin_message_is_explicit_and_mapped():
    s = format_error_line(
        "board rp2040_left_arm: missing virtual_pin indices [2, 5] "
        "(no actuator mapped to these virtual_pin values)"
    )
    o = json.loads(s)
    assert o["field"] == "boards.rp2040_left_arm.virtual_pin"
    assert o["field_path"] == ["boards", "rp2040_left_arm", "virtual_pin"]
    assert "missing virtual_pin indices [2, 5]" in o["message"]
    assert "no actuator mapped" in o["message"]

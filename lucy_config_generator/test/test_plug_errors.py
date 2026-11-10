from lucy_config_generator.plug import header_constraint_errors
from lucy_config_generator.plug import i2c_device_errors
from lucy_config_generator.plug import pca_channel_errors
from lucy_config_generator.plug import resolve_plug
import pytest

BOARD = {"i2c_devices": [{"id": "pca_a", "type": "pca9685", "address": 64}]}


@pytest.mark.parametrize(
    "plug,message",
    [
        ("Servo1", "plug must be a mapping"),
        ({"kind": "laser"}, "unsupported plug kind 'laser'"),
        ({"kind": "servo_header"}, "plug.header is required"),
        ({"kind": "pca9685", "channel": 1}, "plug.device is required"),
        ({"kind": "pca9685", "device": "pca_a"}, "plug.channel is required"),
        ({"kind": "pca9685", "device": "pca_a", "channel": 16}, "out of range 0..15"),
        (
            {"kind": "pca9685", "device": "pca_b", "channel": 1},
            "pca_b is not on this board",
        ),
    ],
)
def test_bad_plug_is_refused_with_its_reason(plug, message):
    with pytest.raises(ValueError, match=message):
        resolve_plug({"plug": plug}, BOARD)


def test_header_and_physical_pin_must_agree():
    plug = {"kind": "servo_header", "header": 3}
    with pytest.raises(ValueError, match="physical_pin must equal plug.header"):
        resolve_plug({"plug": plug, "physical_pin": 4})


def test_unresolvable_actuator_is_reported_by_id_in_header_checks():
    errors = header_constraint_errors("left", [{"id": "jaw", "plug": "x"}])
    assert errors == ["actuator jaw: plug must be a mapping"]


def _i2c(*devices):
    return i2c_device_errors("left", {"i2c_devices": list(devices)})


def test_missing_i2c_devices_is_valid_but_a_non_list_is_not():
    assert i2c_device_errors("left", {}) == []
    assert i2c_device_errors("left", {"i2c_devices": "pca"}) == [
        "board left: i2c_devices must be a list"
    ]


def test_each_i2c_device_needs_a_mapping_and_an_id():
    assert _i2c("pca", {"type": "pca9685", "address": 64}) == [
        "board left: i2c device must be a mapping",
        "board left: i2c device id is required",
    ]


def test_i2c_device_type_address_and_channels_are_checked():
    assert _i2c(
        {"id": "a", "type": "pca9685", "address": 64},
        {"id": "a", "type": "ads1115", "address": 64},
        {"id": "b", "type": "pca9685", "address": "x"},
        {"id": "c", "type": "pca9685", "address": 0x20},
        {"id": "d", "type": "pca9685", "address": 65, "channels": "many"},
        {"id": "e", "type": "pca9685", "address": 66, "channels": 8},
    ) == [
        "board left: duplicate i2c device id a",
        "board left: unsupported i2c device type 'ads1115'",
        "board left: duplicate i2c address 64",
        "board left: i2c address must be an integer",
        "board left: address 32 outside 0x40..0x7F",
        "board left: channels must be 16",
        "board left: channels other than 16",
    ]


def test_two_actuators_on_one_pca_channel_are_reported():
    actuators = [
        {"id": "a", "plug": {"kind": "pca9685", "device": "pca_a", "channel": 2}},
        {"id": "b", "plug": {"kind": "pca9685", "device": "pca_a", "channel": 2}},
        {"id": "c", "physical_pin": 5},
        {"id": "d", "plug": {"kind": "pca9685", "device": "pca_z", "channel": 2}},
    ]
    assert pca_channel_errors("left", BOARD, actuators) == [
        "board left: duplicate PCA channel pca_a:2 (a and b)",
        "actuator d: PCA device pca_z is not on this board",
    ]

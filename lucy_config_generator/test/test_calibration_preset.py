# Copyright 2026 Lucy Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

"""Tests for lucy_config_generator.calibration.preset."""

from __future__ import annotations

from calibration_fixtures import CONFIG
from calibration_fixtures import REAL_ARM
from lucy_config_generator.calibration import generated_limit_mismatches
from lucy_config_generator.calibration import homing_offsets_by_actuator
from lucy_config_generator.calibration import set_calibration_block
from lucy_config_generator.calibration import urdf_limits_from_config
import yaml


def test_homing_offsets_are_keyed_by_actuator():
    offsets = homing_offsets_by_actuator(REAL_ARM, CONFIG)
    assert offsets["pitch"] == -1154
    assert offsets["jaw"] == 1389


def test_records_without_a_homing_offset_are_skipped():
    record = {"id": 1, "range_min": 1, "range_max": 2}
    assert homing_offsets_by_actuator({"m": record}, CONFIG) == {}


def test_calibration_block_is_added_and_replaced_without_touching_the_rest():
    text = "# keep me\nversion: 1\nactuators: []\n"
    once = set_calibration_block(
        text, calibrated_at="T1", homing_offsets={"a": 1}, adapter_serial="S1"
    )
    twice = set_calibration_block(
        once, calibrated_at="T2", homing_offsets={"a": -2}, adapter_serial="S2"
    )
    assert twice.startswith("# keep me\nversion: 1\nactuators: []\n")
    assert twice.count("calibration:") == 1
    data = yaml.safe_load(twice)
    assert data["calibration"] == {
        "adapter_serial": "S2",
        "calibrated_at": "T2",
        "homing_offsets": {"a": -2},
    }


def test_calibration_block_keeps_keys_after_it():
    text = "calibration:\n  adapter_serial: old\n  homing_offsets: {}\nsensors: []\n"
    out = set_calibration_block(text, calibrated_at="T", homing_offsets={})
    assert yaml.safe_load(out)["sensors"] == []
    assert yaml.safe_load(out)["calibration"]["adapter_serial"] == ""


_JAW_ACTUATOR = {
    "id": "jaw",
    "urdf_joint": "Jaw",
    "limit_min": 0.0,
    "limit_max": 2.0,
    "offset": 1.0,
    "direction": 1,
    "scale": 1,
}


def test_urdf_limits_from_config_maps_each_mapped_actuator():
    limits = urdf_limits_from_config({"actuators": [_JAW_ACTUATOR]})
    assert limits == {"Jaw": (-1.0, 1.0)}


def test_urdf_limits_from_config_skips_actuators_with_no_urdf_joint():
    actuator = dict(_JAW_ACTUATOR, urdf_joint="")
    assert urdf_limits_from_config({"actuators": [actuator]}) == {}


_JAW_XACRO = (
    '<joint name="Jaw">'
    '<param name="offset">1</param>'
    '<param name="limit_min">0</param>'
    '<param name="limit_max">2</param>'
    '<param name="min">-1</param>'
    '<param name="max">1</param>'
    "</joint>"
)


def test_generated_limit_mismatches_finds_nothing_wrong_when_values_match():
    config = {"actuators": [_JAW_ACTUATOR]}
    assert generated_limit_mismatches(_JAW_XACRO, config) == []


def test_generated_limit_mismatches_flags_a_missing_joint():
    config = {"actuators": [_JAW_ACTUATOR]}
    problems = generated_limit_mismatches("<robot></robot>", config)
    assert problems == ["Jaw: not in the generated xacro"]


def test_generated_limit_mismatches_flags_a_missing_param():
    xacro = _JAW_XACRO.replace('<param name="min">-1</param>', "")
    problems = generated_limit_mismatches(xacro, {"actuators": [_JAW_ACTUATOR]})
    assert problems == ["Jaw: no min param"]


def test_generated_limit_mismatches_flags_a_wrong_value():
    xacro = _JAW_XACRO.replace(
        '<param name="offset">1</param>', '<param name="offset">5</param>'
    )
    problems = generated_limit_mismatches(xacro, {"actuators": [_JAW_ACTUATOR]})
    assert problems == ["Jaw: offset is 5, expected 1"]


def test_generated_limit_mismatches_skips_actuators_with_no_urdf_joint():
    actuator = dict(_JAW_ACTUATOR, urdf_joint="")
    assert (
        generated_limit_mismatches("<robot></robot>", {"actuators": [actuator]}) == []
    )

# Copyright 2026 Lucy Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

"""Tests for lucy_config_generator.calibration.preset."""

from __future__ import annotations

from calibration_fixtures import CONFIG
from calibration_fixtures import REAL_ARM
import yaml

from lucy_config_generator.calibration import homing_offsets_by_actuator
from lucy_config_generator.calibration import set_calibration_block


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

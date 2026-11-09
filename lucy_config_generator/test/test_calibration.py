# Copyright 2026 Lucy Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

"""Shared calibration logic, including values from a real SO-ARM101."""

from __future__ import annotations

import math

from lucy_config_generator.calibration import CalibrationError
from lucy_config_generator.calibration import homing_offsets_by_actuator
from lucy_config_generator.calibration import plan
from lucy_config_generator.calibration import set_calibration_block
from lucy_config_generator.calibration import ticks_to_radians
from lucy_config_generator.calibration import validate_calibration
from lucy_config_generator.calibration import window_around_centre
from lucy_config_generator.schema import _validate_calibration_block
import pytest
import yaml

# From lerobot-calibrate on a real arm.
REAL_ARM = {
    "shoulder_pan": {
        "id": 1,
        "drive_mode": 0,
        "homing_offset": 1643,
        "range_min": 753,
        "range_max": 3172,
    },
    "shoulder_lift": {
        "id": 2,
        "drive_mode": 0,
        "homing_offset": -1154,
        "range_min": 822,
        "range_max": 3248,
    },
    "elbow_flex": {
        "id": 3,
        "drive_mode": 0,
        "homing_offset": 1124,
        "range_min": 841,
        "range_max": 3072,
    },
    "wrist_flex": {
        "id": 4,
        "drive_mode": 0,
        "homing_offset": 1799,
        "range_min": 853,
        "range_max": 3195,
    },
    "wrist_roll": {
        "id": 5,
        "drive_mode": 0,
        "homing_offset": 446,
        "range_min": 0,
        "range_max": 4095,
    },
    "gripper": {
        "id": 6,
        "drive_mode": 0,
        "homing_offset": 1389,
        "range_min": 2036,
        "range_max": 3541,
    },
}
NAMES = ["rotation", "pitch", "elbow", "wrist_pitch", "wrist_roll", "jaw"]
CONFIG = {
    "actuators": [
        {
            "id": name,
            "urdf_joint": name.title(),
            "physical_pin": i + 1,
            "direction": 1,
            "scale": 1,
        }
        for i, name in enumerate(NAMES)
    ]
}
CENTRE = ticks_to_radians(2048)
MARGIN = math.radians(1.5)


def test_real_arm_travel_crosses_the_encoder_wrap():
    """Why homing is required: unhomed, these windows can't be stored as min <= max."""
    crossing = [
        motor
        for motor, r in REAL_ARM.items()
        if motor != "wrist_roll"
        and (
            r["range_min"] + r["homing_offset"] < 0
            or r["range_max"] + r["homing_offset"] > 4095
        )
    ]
    assert len(crossing) == 5


def test_real_arm_plans_with_every_default_inside_its_limits():
    updates, limits, warnings = plan(REAL_ARM, CONFIG)
    assert warnings == []
    assert len(limits) == 6
    for fields in updates.values():
        assert fields["limit_min"] <= fields["default"] <= fields["limit_max"]


def test_margin_insets_limits_but_not_the_full_turn_joint():
    updates, _, _ = plan(
        REAL_ARM, CONFIG, margin_rad=MARGIN, full_turn_motors=["wrist_roll"]
    )
    assert updates["rotation"]["limit_min"] == pytest.approx(
        ticks_to_radians(753) + MARGIN
    )
    assert updates["wrist_roll"]["limit_min"] == 0


def test_margin_never_pushes_the_centre_out_of_the_window():
    """The jaw rests on its stop at the mid pose, 12 ticks from it."""
    updates, _, _ = plan(REAL_ARM, CONFIG, margin_rad=MARGIN)
    assert updates["jaw"]["limit_min"] == CENTRE


def test_window_keeps_a_centre_just_outside_it():
    """Homing lands on tick 2047, one short of π."""
    assert window_around_centre(1.0, 3.14, CENTRE) == (1.0, CENTRE)
    assert window_around_centre(3.15, 5.0, CENTRE) == (CENTRE, 5.0)


@pytest.mark.parametrize("mode", [0, 1])
def test_known_drive_modes_are_accepted(mode):
    record = dict(REAL_ARM["gripper"], drive_mode=mode)
    assert validate_calibration({"gripper": record})


@pytest.mark.parametrize("mode", [2, -1, "inverted", None])
def test_unknown_drive_mode_is_refused(mode):
    record = dict(REAL_ARM["gripper"], drive_mode=mode)
    with pytest.raises(CalibrationError, match="unknown drive_mode"):
        validate_calibration({"gripper": record})


def test_drive_mode_leaves_direction_alone():
    """Direction also carries the URDF's sign, so drive_mode can't decide it alone."""
    config = {
        "actuators": [{"id": "jaw", "physical_pin": 6, "direction": -1, "scale": 1}]
    }
    updates, _, _ = plan({"gripper": REAL_ARM["gripper"]}, config)
    assert "direction" not in updates["jaw"]


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


@pytest.mark.parametrize(
    "block",
    [
        "nope",
        {"adapter_serial": 5, "homing_offsets": {}},
        {"adapter_serial": "A"},
        {"adapter_serial": "A", "homing_offsets": {"a": "x"}},
        {"adapter_serial": "A", "homing_offsets": {"a": True}},
    ],
)
def test_schema_rejects_a_malformed_block(block):
    with pytest.raises(ValueError):
        _validate_calibration_block(block)


def test_schema_accepts_a_block_without_a_serial():
    _validate_calibration_block({"adapter_serial": "", "homing_offsets": {"a": -5}})

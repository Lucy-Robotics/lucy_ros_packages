# Copyright 2026 Lucy Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

"""Tests for lucy_config_generator.calibration.planning."""

from __future__ import annotations

import math

from calibration_fixtures import CENTRE
from calibration_fixtures import CONFIG
from calibration_fixtures import REAL_ARM
from lucy_config_generator.calibration import CalibrationError
from lucy_config_generator.calibration import plan
from lucy_config_generator.calibration import ticks_to_radians
from lucy_config_generator.calibration.planning import sensor_updates
import pytest

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


def test_drive_mode_leaves_direction_alone():
    """Direction also carries the URDF's sign, so drive_mode can't decide it alone."""
    config = {
        "actuators": [{"id": "jaw", "physical_pin": 6, "direction": -1, "scale": 1}]
    }
    updates, _, _ = plan({"gripper": REAL_ARM["gripper"]}, config)
    assert "direction" not in updates["jaw"]


def test_inverted_drive_mode_warns_to_check_direction():
    record = dict(REAL_ARM["gripper"], drive_mode=1)
    _, _, warnings = plan({"gripper": record}, CONFIG)
    assert any("drive_mode=1" in w for w in warnings)


def test_unmatched_servo_id_is_refused():
    record = dict(REAL_ARM["gripper"], id=99)
    with pytest.raises(CalibrationError, match="matches no actuator"):
        plan({"gripper": record}, CONFIG)


def test_servo_id_matching_several_actuators_is_refused():
    config = {
        "actuators": [
            {"id": "a", "physical_pin": 6, "direction": 1, "scale": 1},
            {"id": "b", "physical_pin": 6, "direction": 1, "scale": 1},
        ]
    }
    with pytest.raises(CalibrationError, match="matches several actuators"):
        plan({"gripper": REAL_ARM["gripper"]}, config)


def test_actuator_with_no_calibration_record_warns_and_is_left_untouched():
    config = {
        "actuators": [
            {"id": "jaw", "physical_pin": 6, "direction": 1, "scale": 1},
            {"id": "extra", "physical_pin": 7, "direction": 1, "scale": 1},
        ]
    }
    updates, _, warnings = plan({"gripper": REAL_ARM["gripper"]}, config)
    assert "extra" not in updates
    assert any("no calibration record for: extra" in w for w in warnings)


def test_sensor_updates_mirrors_its_actuators_window():
    updates = {"jaw": {"limit_min": -1.0, "limit_max": 1.0, "default": 0.0}}
    config = {"sensors": [{"id": "jaw_sensor", "associated_actuator": "jaw"}]}
    assert sensor_updates(config, updates) == {
        "jaw_sensor": {"min_value": -1.0, "max_value": 1.0}
    }


def test_sensor_updates_skips_sensors_with_no_matching_actuator():
    config = {"sensors": [{"id": "jaw_sensor", "associated_actuator": "missing"}]}
    assert sensor_updates(config, {}) == {}


def test_sensor_updates_with_no_sensors_in_config():
    assert sensor_updates({}, {}) == {}

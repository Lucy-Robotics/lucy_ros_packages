# Copyright 2026 Lucy Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

"""Tests for lucy_config_generator.calibration.planning."""

from __future__ import annotations

import math

from calibration_fixtures import CENTRE
from calibration_fixtures import CONFIG
from calibration_fixtures import REAL_ARM
import pytest

from lucy_config_generator.calibration import plan
from lucy_config_generator.calibration import ticks_to_radians

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

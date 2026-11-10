# Copyright 2026 Lucy Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

"""Tests for lucy_config_generator.calibration.ticks."""

from __future__ import annotations

from calibration_fixtures import CENTRE

from lucy_config_generator.calibration import CalibrationError
from lucy_config_generator.calibration import servo_window_to_joint_limits
from lucy_config_generator.calibration import window_around_centre
import pytest


def test_window_keeps_a_centre_just_outside_it():
    """Homing lands on tick 2047, one short of π."""
    assert window_around_centre(1.0, 3.14, CENTRE) == (1.0, CENTRE)
    assert window_around_centre(3.15, 5.0, CENTRE) == (CENTRE, 5.0)


def test_servo_window_to_joint_limits_refuses_a_zero_direction_or_scale():
    with pytest.raises(CalibrationError, match="direction\\*scale must be non-zero"):
        servo_window_to_joint_limits(0.0, 1.0, 0.0, direction=0, scale=1)

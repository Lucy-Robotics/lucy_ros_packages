# Copyright 2026 Lucy Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

"""Tests for lucy_config_generator.calibration.validation."""

from __future__ import annotations

from calibration_fixtures import REAL_ARM
from lucy_config_generator.calibration import CalibrationError
from lucy_config_generator.calibration import validate_calibration
import pytest


@pytest.mark.parametrize("mode", [0, 1])
def test_known_drive_modes_are_accepted(mode):
    record = dict(REAL_ARM["gripper"], drive_mode=mode)
    assert validate_calibration({"gripper": record})


@pytest.mark.parametrize("mode", [2, -1, "inverted", None])
def test_unknown_drive_mode_is_refused(mode):
    record = dict(REAL_ARM["gripper"], drive_mode=mode)
    with pytest.raises(CalibrationError, match="unknown drive_mode"):
        validate_calibration({"gripper": record})

# Copyright 2026 Lucy Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

"""Tests for lucy_config_generator.calibration.ticks."""

from __future__ import annotations

from calibration_fixtures import CENTRE

from lucy_config_generator.calibration import window_around_centre


def test_window_keeps_a_centre_just_outside_it():
    """Homing lands on tick 2047, one short of π."""
    assert window_around_centre(1.0, 3.14, CENTRE) == (1.0, CENTRE)
    assert window_around_centre(3.15, 5.0, CENTRE) == (CENTRE, 5.0)

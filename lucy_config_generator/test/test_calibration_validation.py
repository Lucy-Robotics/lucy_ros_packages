# Copyright 2026 Lucy Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

"""Tests for lucy_config_generator.calibration.validation."""

from __future__ import annotations

from calibration_fixtures import REAL_ARM
from lucy_config_generator.calibration import CalibrationError
from lucy_config_generator.calibration import load_calibration
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


@pytest.mark.parametrize("data", [{}, "nope", None, []])
def test_non_empty_object_is_required(data):
    with pytest.raises(CalibrationError, match="expected a non-empty object"):
        validate_calibration(data)


def test_a_record_that_is_not_an_object_is_refused():
    with pytest.raises(CalibrationError, match="'gripper' is not an object"):
        validate_calibration({"gripper": "nope"})


def test_a_record_missing_required_keys_is_refused():
    with pytest.raises(CalibrationError, match="is missing"):
        validate_calibration({"gripper": {"id": 6}})


def test_an_inverted_range_is_refused():
    record = dict(REAL_ARM["gripper"], range_min=100, range_max=50)
    with pytest.raises(CalibrationError, match="range_min 100 >= range_max 50"):
        validate_calibration({"gripper": record})


def test_load_calibration_reads_and_validates_a_json_file(tmp_path):
    path = tmp_path / "calibration.json"
    path.write_text('{"gripper": {"id": 6, "range_min": 1, "range_max": 2}}')
    assert load_calibration(path)["gripper"]["id"] == 6


def test_load_calibration_refuses_an_unreadable_path(tmp_path):
    with pytest.raises(CalibrationError, match="cannot read calibration"):
        load_calibration(tmp_path / "missing.json")


def test_load_calibration_refuses_invalid_json(tmp_path):
    path = tmp_path / "calibration.json"
    path.write_text("not json")
    with pytest.raises(CalibrationError, match="cannot read calibration"):
        load_calibration(path)

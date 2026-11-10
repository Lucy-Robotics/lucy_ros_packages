# Copyright 2026 Lucy Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

"""Tests for lucy_config_generator.calibration.rewrite."""

from __future__ import annotations

from lucy_config_generator.calibration import apply_to_file
from lucy_config_generator.calibration import apply_to_text
from lucy_config_generator.calibration import apply_updates
from lucy_config_generator.calibration import apply_urdf_limits
from lucy_config_generator.calibration import CalibrationError
import pytest

_YAML = """\
actuators:
  - id: rotation
    offset: 0.0
    limit_min: -1.0
    limit_max: 1.0
    default: 0.0
  - id: pitch
    offset: 0.0
    limit_min: -1.0
    limit_max: 1.0
    default: 0.0
"""

_XACRO = """\
<robot>
  <joint name="Jaw" type="revolute">
    <axis xyz="0 0 1"/>
    <limit effort="1" lower="0" upper="1" velocity="1"/>
  </joint>
</robot>
"""


def test_apply_updates_rewrites_only_the_named_entry():
    out = apply_updates(_YAML, {"rotation": {"offset": 0.5, "limit_min": -0.5}})
    assert "offset: 0.5" in out
    assert "limit_min: -0.5" in out
    assert out.count("offset: 0.0") == 1  # pitch untouched


def test_apply_updates_refuses_an_unknown_entry_id():
    with pytest.raises(CalibrationError, match="no .* entry found"):
        apply_updates(_YAML, {"missing": {"offset": 1.0}})


def test_apply_updates_refuses_a_key_not_present_exactly_once():
    with pytest.raises(CalibrationError, match='expected one "bogus:" line'):
        apply_updates(_YAML, {"rotation": {"bogus": 1.0}})


def test_apply_urdf_limits_rewrites_lower_and_upper():
    out = apply_urdf_limits(_XACRO, {"Jaw": (-1.0, 1.0)})
    assert 'lower="-1"' in out
    assert 'upper="1"' in out


def test_apply_urdf_limits_refuses_a_joint_with_no_limit():
    with pytest.raises(CalibrationError, match="no <limit> found"):
        apply_urdf_limits('<robot><joint name="Other"/></robot>', {"Other": (0.0, 1.0)})


def test_apply_urdf_limits_refuses_a_limit_missing_lower_or_upper():
    xacro = """\
<robot>
  <joint name="Jaw">
    <limit effort="1" velocity="1"/>
  </joint>
</robot>
"""
    with pytest.raises(CalibrationError, match="expected one lower and one upper"):
        apply_urdf_limits(xacro, {"Jaw": (-1.0, 1.0)})


def test_apply_to_text_plans_and_rewrites_in_one_pass():
    calibration = {
        "rotation": {"id": 1, "range_min": 1024, "range_max": 3072},
    }
    config_yaml = """\
actuators:
  - id: rotation
    physical_pin: 1
    offset: 0.0
    limit_min: 0.0
    limit_max: 0.0
    default: 0.0
"""
    text, urdf_limits, warnings = apply_to_text(config_yaml, calibration)
    assert "offset:" in text
    assert urdf_limits == {}
    assert warnings == []


def test_apply_to_file_reads_and_rewrites(tmp_path):
    path = tmp_path / "hardware.yaml"
    path.write_text(
        """\
actuators:
  - id: rotation
    physical_pin: 1
    offset: 0.0
    limit_min: 0.0
    limit_max: 0.0
    default: 0.0
""",
        encoding="utf-8",
    )
    calibration = {"rotation": {"id": 1, "range_min": 1024, "range_max": 3072}}
    text, _limits, _warnings = apply_to_file(path, calibration)
    assert "- id: rotation" in text

# Copyright 2026 Lucy Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

"""Tests for the schema's ``calibration:`` block validator."""

from __future__ import annotations

from pathlib import Path

from lucy_config_generator.schema import _validate_calibration_block
from lucy_config_generator.schema import validate_hardware_yaml
import pytest
import yaml

_FIXTURES = Path(__file__).resolve().parent / "fixtures"


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


def test_validate_hardware_yaml_checks_an_included_calibration_block():
    with (_FIXTURES / "test_mapping.yaml").open(encoding="utf-8") as f:
        data = yaml.safe_load(f)
    data["calibration"] = {"adapter_serial": "S1", "homing_offsets": {"a": 1}}
    validate_hardware_yaml(data)

    data["calibration"] = "nope"
    with pytest.raises(ValueError, match="calibration must be a mapping"):
        validate_hardware_yaml(data)

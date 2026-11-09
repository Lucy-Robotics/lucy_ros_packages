# Copyright 2026 Lucy Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

"""Tests for the schema's ``calibration:`` block validator."""

from __future__ import annotations

import pytest

from lucy_config_generator.schema import _validate_calibration_block


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

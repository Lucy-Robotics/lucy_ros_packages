# Copyright 2026 Lucy Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

from __future__ import annotations


class CalibrationError(RuntimeError):
    """A calibration that cannot be applied to this config."""

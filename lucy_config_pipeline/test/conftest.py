"""Prefer this package's `src/` tree over an older site-packages copy named `src`."""

from __future__ import annotations

from pathlib import Path
import sys

import pytest

_PIPELINE_ROOT = Path(__file__).resolve().parents[1]
if _PIPELINE_ROOT.is_dir():
    sys.path.insert(0, str(_PIPELINE_ROOT))


@pytest.fixture
def rclpy_init_shutdown():
    """Init rclpy for tests that construct ROS nodes."""
    rclpy = pytest.importorskip('rclpy')
    rclpy.init()
    try:
        yield
    finally:
        if rclpy.ok():
            rclpy.shutdown()

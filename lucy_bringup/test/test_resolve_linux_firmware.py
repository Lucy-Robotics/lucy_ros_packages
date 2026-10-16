# Copyright 2026 Lucy Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

"""Unit tests for host linux firmware binary resolution (no ROS launch)."""

from __future__ import annotations

import os
from pathlib import Path
from unittest.mock import patch

import pytest

# Import helpers without executing generate_launch_description ROS wiring.
import importlib.util

_LAUNCH = (
    Path(__file__).resolve().parents[1] / 'launch' / 'lucy.launch.py'
)


def _load_launch_module():
    spec = importlib.util.spec_from_file_location('lucy_bringup_lucy_launch', _LAUNCH)
    assert spec is not None and spec.loader is not None
    mod = importlib.util.module_from_spec(spec)
    # Launch imports need ament; skip if unavailable in bare pytest.
    try:
        spec.loader.exec_module(mod)
    except ModuleNotFoundError as exc:
        pytest.skip(f'ROS launch deps missing: {exc}')
    return mod


def test_resolve_linux_firmware_binary_from_env(tmp_path: Path, monkeypatch):
    mod = _load_launch_module()
    binary = tmp_path / 'lucy-embedded-firmware-linux'
    binary.write_text('#!/bin/sh\n', encoding='utf-8')
    binary.chmod(0o755)
    monkeypatch.setenv('LUCY_EMBEDDED_FIRMWARE_LINUX', str(binary))
    assert mod.resolve_linux_firmware_binary() == binary.resolve()


def test_resolve_linux_firmware_binary_missing_env_raises(monkeypatch):
    mod = _load_launch_module()
    monkeypatch.setenv('LUCY_EMBEDDED_FIRMWARE_LINUX', '/no/such/binary')
    monkeypatch.delenv('PATH', raising=False)
    with patch.object(mod.shutil, 'which', return_value=None):
        with pytest.raises(RuntimeError, match='not a file'):
            mod.resolve_linux_firmware_binary()

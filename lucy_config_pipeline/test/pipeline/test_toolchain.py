# Copyright 2026 Lucy Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

"""Pixi cargo/bin resolution for firmware build tools."""

from __future__ import annotations

import os
from pathlib import Path
import stat

import pytest

from src.pipeline import toolchain


def _make_exec(path: Path) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('#!/bin/sh\n', encoding='utf-8')
    path.chmod(path.stat().st_mode | stat.S_IXUSR)
    return path


def test_firmware_tool_env_prepends_conda_cargo_bin(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
):
    prefix = tmp_path / 'env'
    monkeypatch.setenv('CONDA_PREFIX', str(prefix))
    monkeypatch.setenv('PATH', '/usr/bin')
    env = toolchain.firmware_tool_env({'PATH': '/usr/bin', 'OTHER': '1'})
    cargo_bin = str(prefix / 'cargo' / 'bin')
    assert env['CARGO_HOME'] == str(prefix / 'cargo')
    assert env['RUSTUP_HOME'] == str(prefix / 'rustup')
    assert env['PATH'].split(os.pathsep)[0] == cargo_bin
    assert '/usr/bin' in env['PATH']
    assert env['OTHER'] == '1'


def test_resolve_elf2uf2_uses_conda_cargo_bin(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
):
    prefix = tmp_path / 'env'
    monkeypatch.setenv('CONDA_PREFIX', str(prefix))
    monkeypatch.delenv('PATH', raising=False)
    binary = _make_exec(prefix / 'cargo' / 'bin' / 'elf2uf2-rs')
    resolved = toolchain.resolve_firmware_tool('elf2uf2-rs')
    assert resolved == binary


def test_resolve_elf2uf2_missing_mentions_firmware_setup(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
):
    prefix = tmp_path / 'empty-env'
    prefix.mkdir()
    monkeypatch.setenv('CONDA_PREFIX', str(prefix))
    monkeypatch.setenv('PATH', str(tmp_path / 'nowhere'))
    with pytest.raises(toolchain.FirmwareToolchainError, match='pixi run firmware-setup'):
        toolchain.resolve_firmware_tool('elf2uf2-rs')


def test_resolve_prefers_cargo_bin_over_stale_path_entry(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
):
    """Definite fix for Errno 2: do not depend on whichever elf2uf2 happens to be on PATH."""
    prefix = tmp_path / 'env'
    monkeypatch.setenv('CONDA_PREFIX', str(prefix))
    correct = _make_exec(prefix / 'cargo' / 'bin' / 'elf2uf2-rs')
    decoy_dir = tmp_path / 'decoy'
    decoy_dir.mkdir()
    # Decoy is first on PATH but must lose to $CONDA_PREFIX/cargo/bin.
    monkeypatch.setenv('PATH', str(decoy_dir))
    resolved = toolchain.resolve_firmware_tool('elf2uf2-rs')
    assert resolved == correct

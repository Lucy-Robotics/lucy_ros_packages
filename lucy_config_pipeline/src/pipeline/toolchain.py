# Copyright 2026 Lucy Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

"""Resolve the Pixi-hosted Rust firmware toolchain for cargo / elf2uf2-rs.

``pixi run firmware-setup`` installs rustup + ``elf2uf2-rs`` under
``$CONDA_PREFIX/cargo`` (not ``$CONDA_PREFIX/bin``). The config pipeline node
inherits the Pixi env ``bin`` on ``PATH`` but not ``cargo/bin``, so bare
``elf2uf2-rs`` fails with ``[Errno 2] No such file or directory`` even after a
successful setup. Always prepend ``cargo/bin`` and prefer absolute tool paths.
"""

from __future__ import annotations

import os
from pathlib import Path
import shutil


class FirmwareToolchainError(RuntimeError):
    """Firmware host tools are missing or not on the Pixi cargo PATH."""


def conda_prefix() -> Path | None:
    raw = (os.environ.get('CONDA_PREFIX') or '').strip()
    return Path(raw) if raw else None


def cargo_bin_dir(prefix: Path | None = None) -> Path | None:
    root = prefix if prefix is not None else conda_prefix()
    if root is None:
        return None
    return root / 'cargo' / 'bin'


def firmware_tool_env(base: dict[str, str] | None = None) -> dict[str, str]:
    """Copy ``base`` (or ``os.environ``) with Pixi ``CARGO_HOME`` / ``cargo/bin`` on ``PATH``."""
    env = dict(base if base is not None else os.environ)
    prefix = conda_prefix()
    if prefix is None:
        return env

    cargo_home = prefix / 'cargo'
    rustup_home = prefix / 'rustup'
    cargo_bin = cargo_home / 'bin'
    env['CARGO_HOME'] = str(cargo_home)
    env['RUSTUP_HOME'] = str(rustup_home)

    sep = ';' if os.name == 'nt' else ':'
    cargo_bin_s = str(cargo_bin)
    path = env.get('PATH', '')
    parts = [p for p in path.split(sep) if p]
    if cargo_bin_s not in parts:
        env['PATH'] = sep.join([cargo_bin_s, *parts]) if parts else cargo_bin_s
    return env


def _tool_name(name: str) -> str:
    return f'{name}.exe' if os.name == 'nt' else name


def resolve_firmware_tool(name: str, *, env: dict[str, str] | None = None) -> Path:
    """Absolute path to ``cargo`` / ``elf2uf2-rs`` under the Pixi cargo home or ``PATH``.

    Raises:
        FirmwareToolchainError: when the tool cannot be found, with a fix hint.
    """
    exe = _tool_name(name)
    prefix = conda_prefix()
    cargo_bin = cargo_bin_dir(prefix)
    if cargo_bin is not None:
        candidate = cargo_bin / exe
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return candidate

    search_env = firmware_tool_env(env)
    which = shutil.which(exe, path=search_env.get('PATH'))
    if which:
        return Path(which)

    hint_prefix = str(prefix) if prefix is not None else '(CONDA_PREFIX unset)'
    raise FirmwareToolchainError(
        f'{name} not found under {hint_prefix}/cargo/bin (or PATH). '
        f'Run: pixi run firmware-setup  — then restart Core / the config pipeline '
        f'so the node picks up the toolchain. Expected binary: '
        f'{cargo_bin / exe if cargo_bin is not None else exe}'
    )

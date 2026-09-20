"""Firmware toolchain readiness for hardware ACTIVATE / BUILD / FLASH."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path


SETUP_HINT = (
    'Firmware toolchain not ready. Run: pixi run firmware-setup '
    '(or re-run: python3 install.py)'
)


def _find_workspace_root() -> Path:
    """Locate lucy_ws by walking up until ``pixi.toml`` + firmware setup script exist."""
    for parent in Path(__file__).resolve().parents:
        if (parent / 'pixi.toml').is_file() and (
            parent / 'scripts' / 'firmware_setup.py'
        ).is_file():
            return parent
    raise FileNotFoundError(
        'workspace root (pixi.toml + scripts/firmware_setup.py) not found; '
        'cannot verify firmware toolchain'
    )


def _load_firmware_setup_module():
    """Load workspace ``scripts/firmware_setup.py`` without installing a package."""
    script = _find_workspace_root() / 'scripts' / 'firmware_setup.py'

    mod_name = 'lucy_firmware_setup_check'
    spec = importlib.util.spec_from_file_location(mod_name, script)
    if spec is None or spec.loader is None:
        raise ImportError(f'cannot load {script}')
    module = importlib.util.module_from_spec(spec)
    sys.modules[mod_name] = module
    spec.loader.exec_module(module)
    return module


def check_firmware_toolchain_errors() -> list[str]:
    """Return missing toolchain items (empty = ready)."""
    module = _load_firmware_setup_module()
    return list(module.check_firmware_toolchain())


def require_firmware_toolchain() -> None:
    """Raise ``RuntimeError`` when the Rust/RP2040 toolchain is incomplete."""
    errors = check_firmware_toolchain_errors()
    if not errors:
        return
    detail = '; '.join(errors)
    raise RuntimeError(f'{SETUP_HINT}. Missing: {detail}')

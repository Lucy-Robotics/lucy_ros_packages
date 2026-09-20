"""Firmware toolchain readiness for hardware ACTIVATE / BUILD / FLASH."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path


SETUP_HINT = (
    'Firmware toolchain not ready. Run: pixi run firmware-setup '
    '(or re-run: python3 install.py)'
)


def _load_firmware_setup_module():
    """Load workspace ``scripts/firmware_setup.py`` without installing a package."""
    # .../lucy_ws/src/lucy_ros_packages/lucy_config_pipeline/src/pipeline/this.py
    # parents: pipeline, src, lucy_config_pipeline, lucy_ros_packages, src, lucy_ws
    workspace_root = Path(__file__).resolve().parents[5]
    script = workspace_root / 'scripts' / 'firmware_setup.py'
    if not script.is_file():
        # Fallback: walk up looking for scripts/firmware_setup.py
        here = Path(__file__).resolve()
        for parent in here.parents:
            candidate = parent / 'scripts' / 'firmware_setup.py'
            if candidate.is_file():
                script = candidate
                break
        else:
            raise FileNotFoundError(
                'scripts/firmware_setup.py not found; cannot verify firmware toolchain'
            )

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

"""Unit tests for lucy_config_pipeline firmware toolchain helper (no ROS)."""

from __future__ import annotations

from pathlib import Path
import sys
from unittest.mock import MagicMock
from unittest.mock import patch

import pytest

# Make `src.pipeline.*` importable like other pipeline tests.
_PKG = Path(__file__).resolve().parents[2]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))


def test_require_firmware_toolchain_raises_with_hint():
    from src.pipeline.firmware_toolchain import require_firmware_toolchain

    with patch(
        'src.pipeline.firmware_toolchain.check_firmware_toolchain_errors',
        return_value=['cargo not found'],
    ):
        with pytest.raises(RuntimeError, match='firmware-setup'):
            require_firmware_toolchain()


def test_require_firmware_toolchain_ok_when_empty():
    from src.pipeline.firmware_toolchain import require_firmware_toolchain

    with patch(
        'src.pipeline.firmware_toolchain.check_firmware_toolchain_errors',
        return_value=[],
    ):
        require_firmware_toolchain()


def test_check_loads_workspace_script():
    from src.pipeline import firmware_toolchain as mod

    fake = MagicMock()
    fake.check_firmware_toolchain.return_value = ['rustc not found']
    with patch.object(mod, '_load_firmware_setup_module', return_value=fake):
        assert mod.check_firmware_toolchain_errors() == ['rustc not found']

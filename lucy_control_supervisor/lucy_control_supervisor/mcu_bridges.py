"""Host-side bridges to the Arduino Mega boards of the active hardware YAML."""

from __future__ import annotations

import os
from pathlib import Path
import re
import shutil
import xml.etree.ElementTree as ET

from lucy_config_generator.schema import BOARD_CLASS_I2C_PWM_ONLY
from lucy_config_generator.schema import derive_ros2_node_name
import yaml

MCU_BRIDGE_BIN = 'lucy-embedded-firmware-mcu-bridge'
# The bridge prints one of these per PCA9685 when its state changes.
_PCA_LINE = re.compile(r'PCA9685 (0x[0-9a-f]+) (not answering|answering)')
# The bridge prints `Board <serial> <state>` when its link to the board changes.
_BOARD_LINE = re.compile(r'Board \S*\s(.+)')
_UNREACHABLE = (
    ('not found', 'not found on USB: check its cable and serial_id'),
    ('is being flashed', 'being flashed'),
    ('stopped answering', 'stopped answering: reconnecting'),
    ('not answering', 'on USB but not answering (frozen?): unplug and replug it'),
)


def mega_bridges(hardware_yaml: Path) -> list[tuple[str, str, str, str]]:
    """Return ``(board_id, node_name, serial_id, pca_addresses)`` per Mega board.

    ``pca_addresses`` is the bridge's argument naming the board's PCA9685s, e.g. ``0x41,0x40``.
    """
    if not hardware_yaml.is_file():
        return []
    data = yaml.safe_load(hardware_yaml.read_text(encoding='utf-8')) or {}
    out = []
    for board_id, board in (data.get('boards') or {}).items():
        board = board or {}
        serial = str(board.get('serial_id') or '').strip()
        if board.get('board_class') == BOARD_CLASS_I2C_PWM_ONLY:
            pcas = ','.join(f'{int(a):#04x}' for a in board.get('pca_addresses') or [])
            out.append((board_id, derive_ros2_node_name(board_id), serial, pcas))
    return out


def ros2_control_nodes(urdf_xml: str) -> set[str]:
    """``node_name`` of every ros2_control hardware block in the expanded URDF."""
    params = ET.fromstring(urdf_xml).iterfind('ros2_control/hardware/param[@name="node_name"]')
    return {param.text.strip() for param in params if param.text}


def resolve_bridge_binary(start: Path) -> Path | None:
    """Find the bridge: ``LUCY_MCU_BRIDGE``, ``PATH``, then the workspace build above ``start``."""
    env = os.environ.get('LUCY_MCU_BRIDGE', '').strip()
    if env:
        return Path(env).expanduser() if Path(env).expanduser().is_file() else None
    which = shutil.which(MCU_BRIDGE_BIN)
    if which:
        return Path(which)
    rel = Path('src') / 'lucy_embedded_firmware' / 'target' / 'release' / MCU_BRIDGE_BIN
    for root in [start, *start.parents, Path.cwd(), *Path.cwd().parents]:
        if (root / rel).is_file():
            return root / rel
    return None


def pca_warning(board: str, line: str) -> tuple[str, bool] | None:
    """``(warning, active)`` for a bridge's PCA9685 status line, None for other lines."""
    m = _PCA_LINE.match(line)
    if m is None:
        return None
    message = f'{board} Mega: PCA9685 {m[1]} not answering, its servos cannot move'
    return message, m[2] == 'not answering'


def board_unreachable(line: str) -> str | None:
    """Why a bridge's board is unreachable after this line, '' once it answers, None if unrelated."""
    m = _BOARD_LINE.match(line)
    if m is None:
        return None
    if m[1] == 'answering':
        return ''
    if m[1].startswith('cannot be opened'):
        return m[1].removesuffix(', waiting')
    return next((reason for start, reason in _UNREACHABLE if m[1].startswith(start)), None)

"""Host-side bridges to the Arduino Mega boards of the active hardware YAML."""

from __future__ import annotations

import os
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

from lucy_config_generator.schema import BOARD_CLASS_I2C_PWM_ONLY
from lucy_config_generator.schema import derive_ros2_node_name
import yaml

MCU_BRIDGE_BIN = 'lucy-embedded-firmware-mcu-bridge'


def mega_bridges(hardware_yaml: Path) -> list[tuple[str, str]]:
    """Return ``(node_name, serial_id)`` per Mega board that has a serial_id."""
    if not hardware_yaml.is_file():
        return []
    data = yaml.safe_load(hardware_yaml.read_text(encoding='utf-8')) or {}
    out = []
    for board_id, board in (data.get('boards') or {}).items():
        board = board or {}
        serial = str(board.get('serial_id') or '').strip()
        if board.get('board_class') == BOARD_CLASS_I2C_PWM_ONLY and serial:
            out.append((derive_ros2_node_name(board_id), serial))
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

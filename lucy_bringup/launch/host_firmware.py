# Copyright 2026 Lucy Robotics Team
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

"""
Which host program an activated hardware file starts.

Launch reads the robot package's ``config/hardware/active.yaml`` and asks this
module. A board names its own program (``host.firmware``) and the device label
shown when that program cannot open USB. ``board_class`` stays a description of
the wiring. An empty ``host.firmware`` still starts the linux binary for
``bus_servo_only`` and ``mcu_bridge`` for a board that has a ``serial_id``,
which is how existing files behave.
"""

from __future__ import annotations

import re
from typing import Any

_BOARD_ID = re.compile(r'^rp2040_[a-z][a-z0-9_]*$')

LINUX = 'linux'
MCU_BRIDGE = 'mcu_bridge'


def node_name(board_id: str) -> str | None:
    """Suffix after ``rp2040_``. Same rule as ``derive_ros2_node_name``."""
    if not _BOARD_ID.fullmatch(board_id):
        return None
    return board_id[len('rp2040_'):]


def _u16(value: Any) -> int | None:
    if isinstance(value, bool) or value is None:
        return None
    if isinstance(value, int):
        number = value
    elif isinstance(value, str):
        text = value.strip().lower()
        try:
            number = int(text, 16) if text.startswith('0x') else int(text, 10)
        except ValueError:
            return None
    else:
        return None
    if 0 <= number <= 0xFFFF:
        return number
    return None


def shown_device(node: str, host: dict[str, Any], serial: str) -> str:
    """Label the person can match to a cable. The serial distinguishes two Picos."""
    device = str(host.get('device') or '').strip()
    name = device or node
    if serial and serial not in name:
        return f'{name} ({serial})'
    return name


def _usb_ids(host: dict[str, Any]) -> tuple[int, int] | None:
    usb = host.get('usb')
    if not isinstance(usb, dict):
        return None
    vid = _u16(usb.get('vid'))
    pid = _u16(usb.get('pid'))
    if vid is None or pid is None:
        return None
    return vid, pid


def host_plan(boards: Any) -> tuple[list[dict[str, Any]], list[str]]:
    """
    Return ``(targets, problems)`` for the boards in an activated config.

    A target carries ``firmware``, ``node``, ``device``, and for linux also
    ``vid`` and ``pid``. A problem is a sentence the TUI can show as-is.
    """
    if not isinstance(boards, dict):
        return [], []
    targets: list[dict[str, Any]] = []
    problems: list[str] = []
    for board_id, board in boards.items():
        if not isinstance(board, dict):
            continue
        node = node_name(str(board_id))
        if node is None:
            continue
        host = board.get('host') if isinstance(board.get('host'), dict) else {}
        serial = str(board.get('serial_id') or '').strip()
        device = shown_device(node, host, serial)
        firmware = str(host.get('firmware') or '').strip()
        board_class = str(board.get('board_class') or '').strip()
        if firmware == LINUX or (not firmware and board_class == 'bus_servo_only'):
            ids = _usb_ids(host)
            if ids is None:
                problems.append(
                    f'This robot configuration does not name the USB device for {device}. '
                    'Fix it, then relaunch Lucy.'
                )
                continue
            vid, pid = ids
            targets.append({
                'firmware': LINUX,
                'node': node,
                'device': device,
                'vid': vid,
                'pid': pid,
            })
            continue
        if firmware == MCU_BRIDGE or (not firmware and serial):
            if not serial:
                problems.append(
                    f'This robot configuration does not name the USB serial for {device}. '
                    'Fix it, then relaunch Lucy.'
                )
                continue
            targets.append({
                'firmware': MCU_BRIDGE,
                'node': node,
                'device': device,
                'serial': serial,
            })
            continue
        if firmware:
            problems.append(
                f'This robot configuration asks for firmware {firmware}, which Lucy cannot start. '
                'Fix it, then relaunch Lucy.'
            )
    return targets, problems

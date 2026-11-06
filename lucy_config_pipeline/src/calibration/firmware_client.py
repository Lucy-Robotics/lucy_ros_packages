"""Talk to the host firmware's maintenance socket, the only way to reach the servo bus."""

from __future__ import annotations

import json
import os
from pathlib import Path
import socket
import tempfile

# set_half_turn verifies EEPROM writes on six servos, with a delay after each one.
REPLY_TIMEOUT_S = 10.0


def socket_path(node: str) -> Path:
    """Return where the firmware listens, as in firmwares/linux/src/maintenance.rs."""
    runtime = os.environ.get('XDG_RUNTIME_DIR')
    if runtime:
        return Path(runtime) / 'lucy' / f'{node}.sock'
    return Path(tempfile.gettempdir()) / f'lucy-{os.geteuid()}' / f'{node}.sock'


def records_argument(records: list[dict]) -> str:
    """Encode EEPROM records as ``id:homing_offset:min_limit:max_limit`` entries."""
    return ','.join(
        f"{r['id']}:{r['homing_offset']}:{r['min_limit']}:{r['max_limit']}" for r in records
    )


class FirmwareUnreachable(RuntimeError):
    """The firmware process is not running, or stopped answering."""


class FirmwareBackend:
    """One connection to the firmware, reopened as needed. Not thread-safe."""

    def __init__(self, node: str) -> None:
        self._path = socket_path(node)
        self._sock: socket.socket | None = None
        self._reader = None

    def close(self) -> None:
        if self._reader is not None:
            self._reader.close()
        if self._sock is not None:
            self._sock.close()
        self._sock = None
        self._reader = None

    def _ask(self, line: str) -> dict:
        try:
            if self._sock is None:
                self._sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                self._sock.settimeout(REPLY_TIMEOUT_S)
                self._sock.connect(str(self._path))
                self._reader = self._sock.makefile('r', encoding='utf-8')
            self._sock.sendall(line.encode('utf-8') + b'\n')
            reply = self._reader.readline()
            if not reply:
                raise OSError('connection closed')
            return json.loads(reply)
        except (OSError, ValueError) as e:
            self.close()
            raise FirmwareUnreachable(str(e)) from e

    def status(self) -> dict:
        return self._ask('status')

    def select(self, serial: str) -> dict:
        return self._ask(f'select {serial}')

    def read_homing(self) -> dict:
        return self._ask('read_homing')

    def set_half_turn(self, serial: str) -> dict:
        return self._ask(f'set_half_turn {serial}')

    def restore_homing(self, serial: str, records: list[dict]) -> dict:
        return self._ask(f'restore_homing {serial} {records_argument(records)}')

    def begin_joint(self, _joint_id: int) -> None:
        """Nothing to do: a person moves the real arm."""

"""The socket client, against a local server speaking the firmware's protocol."""

from __future__ import annotations

import json
import socket
import threading

import pytest
from src.calibration.firmware_client import FirmwareBackend
from src.calibration.firmware_client import FirmwareUnreachable
from src.calibration.firmware_client import records_argument
from src.calibration.firmware_client import socket_path


def test_socket_path_follows_the_firmware(monkeypatch, tmp_path):
    monkeypatch.setenv('XDG_RUNTIME_DIR', str(tmp_path))
    assert socket_path('so_arm') == tmp_path / 'lucy' / 'so_arm.sock'


def test_records_are_encoded_for_restore():
    records = [
        {'id': 1, 'homing_offset': -250, 'min_limit': 700, 'max_limit': 3300},
        {'id': 2, 'homing_offset': 0, 'min_limit': 0, 'max_limit': 4095},
    ]
    assert records_argument(records) == '1:-250:700:3300,2:0:0:4095'


@pytest.fixture
def firmware(monkeypatch, tmp_path):
    monkeypatch.setenv('XDG_RUNTIME_DIR', str(tmp_path))
    path = tmp_path / 'lucy'
    path.mkdir()
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(str(path / 'node.sock'))
    server.listen()
    seen: list[str] = []

    def serve():
        conn, _ = server.accept()
        with conn, conn.makefile('r') as lines:
            for line in lines:
                seen.append(line.strip())
                conn.sendall(json.dumps({'ok': True}).encode() + b'\n')

    threading.Thread(target=serve, daemon=True).start()
    yield seen
    server.close()


def test_eeprom_requests_name_the_adapter(firmware):
    backend = FirmwareBackend('node')
    backend.status()
    backend.set_half_turn('ABC')
    backend.restore_homing(
        'ABC', [{'id': 1, 'homing_offset': 5, 'min_limit': 0, 'max_limit': 4095}]
    )
    backend.close()
    assert firmware == ['status', 'set_half_turn ABC', 'restore_homing ABC 1:5:0:4095']


def test_no_firmware_is_a_clean_error(monkeypatch, tmp_path):
    monkeypatch.setenv('XDG_RUNTIME_DIR', str(tmp_path))
    with pytest.raises(FirmwareUnreachable):
        FirmwareBackend('absent').status()

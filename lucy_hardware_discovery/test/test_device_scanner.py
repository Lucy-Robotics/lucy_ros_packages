"""Tests for lucy_hardware_discovery.device_scanner."""

from lucy_hardware_discovery.device_scanner import (
    dumps_json,
    match_boards,
    probe_camera,
)


def test_match_boards_empty_serial():
    result = match_boards({"rp2040_left_arm": ""})
    assert result["boards"]["rp2040_left_arm"]["matched"] is False


def test_match_boards_finds_by_usb_serial(monkeypatch):
    monkeypatch.setattr(
        "lucy_hardware_discovery.device_scanner.list_serial_devices",
        lambda: [
            {
                "path": "/dev/ttyACM1",
                "name": "ttyACM1",
                "serial_hint": "E6617C93E37A6629",
                "resolved": "/dev/ttyACM1",
                "usb_serial": "E6617C93E37A6629",
                "usb_vendor_id": "2e8a",
                "usb_product_id": "000a",
            }
        ],
    )
    result = match_boards({"rp2040_right_arm": "E6617C93E37A6629"})
    assert result["boards"]["rp2040_right_arm"]["matched"] is True
    assert result["boards"]["rp2040_right_arm"]["device"] == "/dev/ttyACM1"


def test_match_boards_finds_by_serial_hint(monkeypatch):
    monkeypatch.setattr(
        "lucy_hardware_discovery.device_scanner.list_serial_devices",
        lambda: [
            {
                "path": "/dev/serial/by-id/usb-Pico_ABCD1234",
                "name": "usb-Pico_ABCD1234",
                "serial_hint": "ABCD1234",
                "resolved": "/dev/ttyACM0",
            }
        ],
    )
    result = match_boards({"rp2040_left_arm": "ABCD1234"})
    assert result["boards"]["rp2040_left_arm"]["matched"] is True
    assert result["boards"]["rp2040_left_arm"]["device"] == (
        "/dev/serial/by-id/usb-Pico_ABCD1234"
    )


def test_probe_camera_missing_device():
    result = probe_camera("/dev/video_this_does_not_exist_999")
    assert result["success"] is False


def test_dumps_json_roundtrip():
    payload = {"devices": [{"device": "/dev/video0"}]}
    text = dumps_json(payload)
    assert "/dev/video0" in text

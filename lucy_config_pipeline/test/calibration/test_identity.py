"""The boot routing table."""

from __future__ import annotations

import pytest
from src.calibration.identity import decide
from src.calibration.identity import valid_serial

SERIAL = "ABC123"
IDS = {"rotation": 1, "pitch": 2}


def firmware(**overrides) -> dict:
    status = {
        "link": "open",
        "open_serial": SERIAL,
        "servos": [{"id": 1, "age_ms": 40}, {"id": 2, "age_ms": 40}],
    }
    status.update(overrides)
    return status


def homing(rotation: int = 11, pitch: int = 22) -> list[dict]:
    return [{"id": 1, "homing_offset": rotation}, {"id": 2, "homing_offset": pitch}]


@pytest.fixture
def calibrated(store):
    text = store.read_named_yaml("default") + (
        "calibration:\n"
        f"  adapter_serial: '{SERIAL}'\n"
        "  homing_offsets: {rotation: 11, pitch: 22}\n"
    )
    store.write_named_yaml("mine", text)
    store.activate("mine")
    return store


def run(store, status=None, offsets=None):
    return decide(
        status=status if status is not None else firmware(),
        store=store,
        homing=offsets if offsets is not None else homing(),
        joint_ids_by_actuator=IDS,
    )


def test_unreachable(store):
    identity = decide(status=None, store=store, homing=None, joint_ids_by_actuator=IDS)
    assert identity.state == "unreachable"
    assert not identity.control_allowed


@pytest.mark.parametrize(
    "link", ["no_adapter", "ambiguous", "selected_missing", "open_failed"]
)
def test_bus_not_open(store, link):
    identity = run(store, firmware(link=link))
    assert identity.state == link
    assert not identity.control_allowed


def test_adapter_without_a_serial_is_refused(store):
    assert run(store, firmware(open_serial=None)).state == "no_serial"


def test_silent_servo_means_bus_dead(store):
    status = firmware(servos=[{"id": 1, "age_ms": 40}, {"id": 2, "age_ms": None}])
    assert run(store, status).state == "bus_dead"


def test_unknown_arm_needs_calibrating(store):
    identity = run(store)
    assert identity.state == "no_binding"
    assert not identity.control_allowed


def test_known_arm_with_its_preset_active_may_be_driven(calibrated):
    identity = run(calibrated)
    assert identity.state == "match"
    assert identity.matched_config == "mine"
    assert identity.config_active is True
    assert identity.control_allowed is True


def test_known_arm_with_another_preset_active_must_activate_first(calibrated):
    calibrated.activate("default")
    identity = run(calibrated)
    assert identity.state == "match"
    assert identity.config_active is False
    assert identity.control_allowed is False


def test_arm_recalibrated_elsewhere_is_caught(calibrated):
    identity = run(calibrated, offsets=homing(rotation=999))
    assert identity.state == "offsets_differ"
    assert not identity.control_allowed


def test_interrupted_session_comes_first(calibrated):
    calibrated.write_journal({"serial": SERIAL, "old": []})
    assert run(calibrated).state == "recovery"


@pytest.mark.parametrize(
    "serial,ok",
    [
        ("5970072960", True),
        ("A B-1", True),
        ("", False),
        ("   ", False),
        ('bad"serial', False),
        ("a;rm -rf", False),
        (None, False),
    ],
)
def test_serial_validation(serial, ok):
    assert valid_serial(serial) is ok

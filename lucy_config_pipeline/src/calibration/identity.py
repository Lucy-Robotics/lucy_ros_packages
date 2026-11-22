"""Decide what a connected arm is, and whether it is safe to drive."""

from __future__ import annotations

from dataclasses import dataclass

from lucy_config_generator.safe_strings import SAFE_STRING
import yaml

from .firmware_client import FirmwareUnreachable
from ..config_store import ConfigStore

# A servo that has not answered for this long is not on the bus any more.
FRESH_READ_MS = 500


@dataclass(frozen=True)
class Identity:
    """What boot routing and the control gate act on."""

    state: str
    adapter_serial: str = ""
    matched_config: str = ""
    config_active: bool = False
    control_allowed: bool = False
    message: str = ""


def valid_serial(serial: object) -> bool:
    """Whether a USB iSerial can identify an arm and be stored in a preset."""
    return (
        isinstance(serial, str)
        and bool(serial.strip())
        and bool(SAFE_STRING.fullmatch(serial))
    )


def servos_fresh(status: dict) -> bool:
    """Whether every servo answered recently. The bus link alone doesn't show this."""
    servos = status.get("servos") or []
    return bool(servos) and all(
        s.get("age_ms") is not None and s["age_ms"] < FRESH_READ_MS for s in servos
    )


_LINK_STATES = {
    "no_adapter": "No USB adapter is connected.",
    "ambiguous": "Several USB adapters are connected: choose which arm to use.",
    "selected_missing": "The chosen USB adapter is not connected.",
    "open_failed": "The USB adapter could not be opened.",
}


def decide(
    *,
    status: dict | None,
    store: ConfigStore,
    homing: list[dict] | None,
    joint_ids_by_actuator: dict[str, int],
) -> Identity:
    """Apply the boot routing table to what the firmware reports."""
    if store.read_journal():
        return Identity(
            "recovery", message="A calibration was interrupted: resume or restore it."
        )
    if status is None:
        return Identity("unreachable", message="The host firmware is not answering.")

    link = status.get("link")
    if link in _LINK_STATES:
        return Identity(link, message=_LINK_STATES[link])
    if link != "open":
        return Identity("unreachable", message="The bus is not open.")
    serial = status.get("open_serial")
    if not valid_serial(serial):
        return Identity(
            "no_serial",
            message="This USB adapter has no usable serial number to identify it by.",
        )
    if not servos_fresh(status):
        return Identity(
            "bus_dead",
            adapter_serial=serial,
            message="Some servos are not answering: check power and wiring.",
        )

    config = store.bound_config(serial)
    if not config:
        return Identity(
            "no_binding",
            adapter_serial=serial,
            message="This arm has not been calibrated here.",
        )

    active = store.get_active_name() == config
    expected = store.read_calibration_block(config).get("homing_offsets") or {}
    if homing is not None and expected:
        found = {r["id"]: r["homing_offset"] for r in homing}
        for actuator, offset in expected.items():
            if found.get(joint_ids_by_actuator.get(actuator)) != offset:
                return Identity(
                    "offsets_differ",
                    adapter_serial=serial,
                    matched_config=config,
                    config_active=active,
                    message="This arm was recalibrated elsewhere: its preset no longer applies.",
                )
    return Identity(
        "match",
        adapter_serial=serial,
        matched_config=config,
        config_active=active,
        control_allowed=active,
        message="Calibrated arm recognised.",
    )


def resolve_identity(
    *, profile: dict | None, real: bool, backend, store: ConfigStore
) -> Identity:
    """Resolve what the calibration node's identity service and control gate both need."""
    if profile is None:
        return Identity("unsupported", control_allowed=True)
    if not real:
        return Identity("simulated", control_allowed=True, config_active=True)
    try:
        status = backend.status()
    except FirmwareUnreachable:
        status = None
    homing = None
    if status is not None and status.get("link") == "open":
        try:
            reply = backend.read_homing()
            homing = reply.get("homing") if reply.get("ok") else None
        except FirmwareUnreachable:
            pass
    return decide(
        status=status,
        store=store,
        homing=homing,
        joint_ids_by_actuator=_actuator_ids(store),
    )


def _actuator_ids(store: ConfigStore) -> dict[str, int]:
    try:
        config = yaml.safe_load(store.read_active_yaml())
    except (OSError, yaml.YAMLError):
        return {}
    return {
        str(a["id"]): int(a["physical_pin"])
        for a in (config or {}).get("actuators") or []
        if a.get("physical_pin") is not None
    }

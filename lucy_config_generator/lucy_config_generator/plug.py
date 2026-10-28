# Copyright 2026 Lucy Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

"""
Resolve and assign the wire that connects an actuator to a header or a PCA9685.

This module is the only reader and writer of that wire. A missing ``plug``
means ``physical_pin`` is the Servo header (legacy presets).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any
from typing import Union

HEADER_MIN = 1
HEADER_MAX = 18
# Servo1/Servo17 share PWM0 A; Servo2/Servo18 share PWM0 B.
SHARED_PWM_PAIRS = ((1, 17), (2, 18))


@dataclass(frozen=True)
class HeaderPlug:
    """One servo plugged into a Servo2040 header (silk Servo1–Servo18)."""

    header: int

    @property
    def kind(self) -> str:
        return 'servo_header'


@dataclass(frozen=True)
class PcaPlug:
    """One servo on a PCA9685 channel attached to this board's Qw/ST bus."""

    device: str
    channel: int

    @property
    def kind(self) -> str:
        return 'pca9685'


ResolvedPlug = Union[HeaderPlug, PcaPlug]


def resolve_plug(
    actuator: dict[str, Any], board: dict[str, Any] | None = None
) -> ResolvedPlug:
    """
    Return the wire for ``actuator``.

    :param actuator: One hardware-YAML actuator mapping.
    :param board: Owning board. Required for PCA device lookup when present.
    :raises ValueError: The ``plug`` mapping is missing, unknown, or disagrees
        with ``physical_pin``.
    :raises TypeError: ``header`` or ``physical_pin`` is not an integer.
    """
    plug = actuator.get('plug')
    if plug is None:
        return HeaderPlug(int(actuator['physical_pin']))
    if not isinstance(plug, dict):
        raise ValueError('plug must be a mapping')
    kind = plug.get('kind')
    if kind == 'pca9685':
        return _resolve_pca(plug, board)
    if kind != 'servo_header':
        raise ValueError(f'unsupported plug kind {kind!r}')
    if 'header' not in plug:
        raise ValueError('plug.header is required')
    header = int(plug['header'])
    if 'physical_pin' in actuator and int(actuator['physical_pin']) != header:
        raise ValueError('physical_pin must equal plug.header')
    return HeaderPlug(header)


def _resolve_pca(plug: dict[str, Any], board: dict[str, Any] | None) -> PcaPlug:
    if 'device' not in plug or not str(plug.get('device') or '').strip():
        raise ValueError('plug.device is required')
    if 'channel' not in plug:
        raise ValueError('plug.channel is required')
    channel = int(plug['channel'])
    if channel < 0 or channel > 15:
        raise ValueError(f'channel {channel} out of range 0..15')
    device = str(plug['device'])
    if board is not None:
        devices = board.get('i2c_devices') or []
        known = {
            str(item.get('id'))
            for item in devices
            if isinstance(item, dict) and item.get('id')
        }
        if device not in known:
            raise ValueError(f'PCA device {device} is not on this board')
    return PcaPlug(device=device, channel=channel)


def assign_plug(actuator: dict[str, Any], header: int) -> None:
    """
    Write a header plug and keep ``physical_pin`` equal to that header.

    :param actuator: Actuator mapping mutated in place.
    :param header: Servo2040 silk number, 1–18.
    """
    header = int(header)
    actuator['plug'] = {'kind': 'servo_header', 'header': header}
    actuator['physical_pin'] = header


def assign_pca_plug(actuator: dict[str, Any], device: str, channel: int) -> None:
    """
    Write a PCA9685 plug. The row does not occupy a Servo header.

    :param actuator: Actuator mapping mutated in place.
    :param device: ``i2c_devices[].id`` on the actuator's board.
    :param channel: PCA channel, 0–15.
    """
    actuator['plug'] = {
        'kind': 'pca9685',
        'device': device,
        'channel': int(channel),
    }
    actuator.pop('physical_pin', None)


def header_constraint_errors(board_id: str, actuators: list[dict[str, Any]]) -> list[str]:
    """
    Return header range, duplicate, and shared-PWM-slice errors for one board.

    :param board_id: Board id used in error text.
    :param actuators: Actuators already known to belong to ``board_id``.
    """
    errors: list[str] = []
    claimed: dict[int, str] = {}
    for actuator in actuators:
        aid = actuator.get('id') or '<unknown>'
        try:
            resolved = resolve_plug(actuator)
        except (KeyError, TypeError, ValueError) as exc:
            errors.append(f'actuator {aid}: {exc}')
            continue
        if not isinstance(resolved, HeaderPlug):
            continue
        header = resolved.header
        if header < HEADER_MIN or header > HEADER_MAX:
            errors.append(
                f'actuator {aid}: header {header} out of range {HEADER_MIN}..{HEADER_MAX}'
            )
            continue
        previous = claimed.get(header)
        if previous is not None:
            errors.append(
                f'board {board_id}: duplicate header {header} ({previous} and {aid})'
            )
            continue
        claimed[header] = str(aid)
    used = set(claimed)
    for left, right in SHARED_PWM_PAIRS:
        if left in used and right in used:
            errors.append(
                f'board {board_id}: Servo{left} and Servo{right} share a PWM slice'
            )
    return errors


def i2c_device_errors(board_id: str, board: dict[str, Any]) -> list[str]:
    """
    Return address, id, and channel-count errors for ``i2c_devices``.

    :param board_id: Board id used in error text.
    :param board: Board mapping. Missing ``i2c_devices`` is valid.
    """
    devices = board.get('i2c_devices')
    if devices is None:
        return []
    if not isinstance(devices, list):
        return [f'board {board_id}: i2c_devices must be a list']
    errors: list[str] = []
    seen_ids: set[str] = set()
    seen_addresses: set[int] = set()
    for item in devices:
        if not isinstance(item, dict):
            errors.append(f'board {board_id}: i2c device must be a mapping')
            continue
        device_id = str(item.get('id') or '')
        if not device_id:
            errors.append(f'board {board_id}: i2c device id is required')
            continue
        if device_id in seen_ids:
            errors.append(f'board {board_id}: duplicate i2c device id {device_id}')
        seen_ids.add(device_id)
        if item.get('type') != 'pca9685':
            errors.append(
                f'board {board_id}: unsupported i2c device type {item.get("type")!r}'
            )
        try:
            address = int(item['address'])
        except (KeyError, TypeError, ValueError):
            errors.append(f'board {board_id}: i2c address must be an integer')
            continue
        if address < 0x40 or address > 0x7F:
            errors.append(f'board {board_id}: address {address} outside 0x40..0x7F')
        elif address in seen_addresses:
            errors.append(f'board {board_id}: duplicate i2c address {address}')
        seen_addresses.add(address)
        try:
            channels = int(item.get('channels', 16))
        except (TypeError, ValueError):
            errors.append(f'board {board_id}: channels must be 16')
            continue
        if channels != 16:
            errors.append(f'board {board_id}: channels other than 16')
    return errors


def pca_channel_errors(
    board_id: str,
    board: dict[str, Any],
    actuators: list[dict[str, Any]],
) -> list[str]:
    """
    Return duplicate PCA channel errors. A PCA plug does not claim a header.

    :param board_id: Board id used in error text.
    :param board: Board mapping, including ``i2c_devices``.
    :param actuators: Actuators on ``board_id``.
    """
    errors: list[str] = []
    claimed: dict[tuple[str, int], str] = {}
    for actuator in actuators:
        aid = actuator.get('id') or '<unknown>'
        try:
            resolved = resolve_plug(actuator, board)
        except (KeyError, TypeError, ValueError) as exc:
            errors.append(f'actuator {aid}: {exc}')
            continue
        if not isinstance(resolved, PcaPlug):
            continue
        key = (resolved.device, resolved.channel)
        previous = claimed.get(key)
        if previous is not None:
            errors.append(
                f'board {board_id}: duplicate PCA channel '
                f'{resolved.device}:{resolved.channel} ({previous} and {aid})'
            )
            continue
        claimed[key] = str(aid)
    return errors

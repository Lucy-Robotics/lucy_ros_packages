# Copyright 2026 Lucy Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

r"""
Turn a LeRobot-shaped servo calibration into hardware YAML and URDF limits.

LeRobot's half-turn homing makes the pose held during calibration read as tick
2048 (π rad on Lucy's 0–2π ↔ 0–4096 map). That pose is the URDF default (joint
0). Lucy therefore stores:

* ``offset`` / ``default`` = π (servo-frame centre)
* ``limit_min`` / ``limit_max`` = travel window in servo-frame radians
* URDF ``lower`` / ``upper`` = the same window mapped to joint space:
  ``joint = (servo - offset) * direction * scale``
"""

from __future__ import annotations

import json
import math
from pathlib import Path
import re
from typing import Any
from typing import Iterable

import yaml

# Firmware / YAML map 0–2π rad linearly onto the STS3215 0–4096 tick span.
TICKS_PER_TURN = 4096
# LeRobot half-turn homing centres Present_Position here (= URDF joint 0).
CENTRE_TICKS = TICKS_PER_TURN // 2

REQUIRED_RECORD_KEYS = ('id', 'range_min', 'range_max')
KNOWN_DRIVE_MODES = (0, 1)
# Generated values are written with six significant digits.
LIMIT_TOLERANCE_RAD = 1e-3


class CalibrationError(RuntimeError):
    """A calibration that cannot be applied to this config."""


def ticks_to_radians(ticks: float) -> float:
    """Convert raw encoder ticks to radians stored in the hardware YAML."""
    return round(ticks * (2.0 * math.pi) / TICKS_PER_TURN, 6)


def servo_window_to_joint_limits(
    limit_min: float,
    limit_max: float,
    offset: float,
    direction: float,
    scale: float,
) -> tuple[float, float]:
    """Map a servo-frame travel window to URDF joint lower/upper (sorted)."""
    denom = direction * scale
    if denom == 0:
        raise CalibrationError('direction*scale must be non-zero')
    a = (limit_min - offset) * denom
    b = (limit_max - offset) * denom
    lo, hi = (a, b) if a <= b else (b, a)
    return round(lo, 6), round(hi, 6)


def window_around_centre(
    limit_min: float,
    limit_max: float,
    centre: float,
    margin: float = 0.0,
) -> tuple[float, float]:
    """
    Inset a travel window by ``margin`` while always keeping ``centre`` inside it.

    The schema requires ``default`` (the centre) within the limits. A joint can
    rest against its own stop at the mid pose, as the jaw does, and homing lands
    on tick 2047 while π is tick 2048.
    """
    return min(limit_min + margin, centre), max(limit_max - margin, centre)


def validate_calibration(data: Any, source: str = 'calibration') -> dict[str, dict]:
    """Reject records that cannot be applied, including an unknown ``drive_mode``."""
    if not isinstance(data, dict) or not data:
        raise CalibrationError(f'{source}: expected a non-empty object of motor records')

    for motor, record in data.items():
        if not isinstance(record, dict):
            raise CalibrationError(f'{source}: {motor!r} is not an object')
        missing = [k for k in REQUIRED_RECORD_KEYS if k not in record]
        if missing:
            raise CalibrationError(f'{source}: {motor!r} is missing {missing}')
        if record['range_min'] >= record['range_max']:
            raise CalibrationError(
                f"{source}: {motor!r} has range_min {record['range_min']} "
                f">= range_max {record['range_max']}"
            )
        drive_mode = record.get('drive_mode', 0)
        if drive_mode not in KNOWN_DRIVE_MODES:
            raise CalibrationError(
                f'{source}: {motor!r} has unknown drive_mode {drive_mode!r}; '
                f'expected one of {KNOWN_DRIVE_MODES}'
            )
    return data


def load_calibration(path: Path) -> dict[str, dict]:
    """Read a lerobot-calibrate JSON, rejecting records that cannot be applied."""
    try:
        data = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as e:
        raise CalibrationError(f'cannot read calibration {path}: {e}') from e
    return validate_calibration(data, str(path))


def plan(
    calibration: dict[str, dict],
    config: dict,
    *,
    margin_rad: float = 0.0,
    full_turn_motors: Iterable[str] = (),
) -> tuple[dict, dict[str, tuple[float, float]], list[str]]:
    """
    Pair calibration records with actuators by servo id, and derive their values.

    Returns per-actuator YAML field updates, per-``urdf_joint`` limit pairs, and
    warnings. Raises when a record matches no actuator or two actuators share
    its id.

    ``margin_rad`` keeps joints off the stops just recorded. Motors in
    ``full_turn_motors`` have no stops, so they get no margin.
    """
    actuators = config.get('actuators') or []
    full_turn = set(full_turn_motors)
    warnings: list[str] = []
    updates: dict[str, dict[str, float]] = {}
    urdf_limits: dict[str, tuple[float, float]] = {}

    by_pin: dict[int, list[dict]] = {}
    for actuator in actuators:
        pin = actuator.get('physical_pin')
        if pin is not None:
            by_pin.setdefault(int(pin), []).append(actuator)

    centre = ticks_to_radians(CENTRE_TICKS)
    for motor, record in sorted(calibration.items(), key=lambda kv: kv[1]['id']):
        servo_id = int(record['id'])
        matched = by_pin.get(servo_id, [])
        if not matched:
            raise CalibrationError(
                f'{motor!r} (servo id {servo_id}) matches no actuator: no '
                f'physical_pin {servo_id} in the config'
            )
        if len(matched) > 1:
            ids = [str(a['id']) for a in matched]
            raise CalibrationError(
                f'{motor!r} (servo id {servo_id}) matches several actuators '
                f"({', '.join(ids)}); physical_pin must be unique"
            )
        actuator = matched[0]
        actuator_id = str(actuator['id'])
        if record.get('drive_mode'):
            warnings.append(
                f"{motor}: drive_mode={record['drive_mode']} means LeRobot drives "
                f'this joint inverted. direction is left at its current value; '
                f'check {actuator_id} against the 3D view.'
            )
        limit_min, limit_max = window_around_centre(
            ticks_to_radians(record['range_min']),
            ticks_to_radians(record['range_max']),
            centre,
            0.0 if motor in full_turn else margin_rad,
        )
        updates[actuator_id] = {
            'offset': centre,
            'limit_min': limit_min,
            'limit_max': limit_max,
            'default': centre,
        }

        urdf_joint = str(actuator.get('urdf_joint') or '').strip()
        if urdf_joint:
            urdf_limits[urdf_joint] = servo_window_to_joint_limits(
                limit_min,
                limit_max,
                centre,
                float(actuator.get('direction', 1)),
                float(actuator.get('scale', 1)),
            )

    unmatched = [str(a['id']) for a in actuators if str(a['id']) not in updates]
    if unmatched:
        warnings.append(
            f"no calibration record for: {', '.join(sorted(unmatched))} "
            f'(left untouched)'
        )
    return updates, urdf_limits, warnings


def sensor_updates(config: dict, updates: dict[str, dict]) -> dict[str, dict]:
    """Mirror each actuator's new window onto the sensors that report on it."""
    out: dict[str, dict[str, float]] = {}
    for sensor in config.get('sensors') or []:
        actuator = str(sensor.get('associated_actuator', ''))
        if actuator in updates:
            out[str(sensor['id'])] = {
                'min_value': updates[actuator]['limit_min'],
                'max_value': updates[actuator]['limit_max'],
            }
    return out


def _block_span(text: str, entry_id: str) -> tuple[int, int]:
    """Character span of the `- id: <entry_id>` list entry, up to the next one."""
    opener = re.search(rf'(?m)^\s*-\s+id:\s+{re.escape(entry_id)}\s*$', text)
    if opener is None:
        raise CalibrationError(f'no "- id: {entry_id}" entry found')
    start = opener.start()
    following = re.search(r'(?m)^\s*-\s+id:\s+\S+\s*$', text[opener.end():])
    end = opener.end() + following.start() if following else len(text)
    return start, end


def apply_updates(text: str, updates: dict[str, dict]) -> str:
    """Rewrite the named keys inside each entry, leaving comments and order alone."""
    for entry_id, fields in updates.items():
        start, end = _block_span(text, entry_id)
        block = text[start:end]
        for key, value in fields.items():
            block, count = re.subn(rf'(?m)^(\s*{key}:).*$', rf'\g<1> {value:g}', block)
            if count != 1:
                raise CalibrationError(
                    f'{entry_id}: expected one "{key}:" line, found {count}'
                )
        text = text[:start] + block + text[end:]
    return text


def apply_urdf_limits(text: str, limits: dict[str, tuple[float, float]]) -> str:
    """Rewrite ``<limit ... lower= upper=>`` on named joints; keep other attrs."""
    for joint, (lo, hi) in limits.items():
        pattern = (
            rf'(<joint\s+name="{re.escape(joint)}"[^>]*>.*?'
            rf'<limit\b)([^>]*)(/>)'
        )

        def _rewrite(match: re.Match[str], *, lower: float = lo, upper: float = hi) -> str:
            attrs = match.group(2)
            attrs, n_lo = re.subn(r'\blower="[^"]*"', f'lower="{lower:g}"', attrs)
            attrs, n_hi = re.subn(r'\bupper="[^"]*"', f'upper="{upper:g}"', attrs)
            if n_lo != 1 or n_hi != 1:
                raise CalibrationError(
                    f'joint {joint!r}: expected one lower and one upper on <limit>, '
                    f'found lower={n_lo} upper={n_hi}'
                )
            return f'{match.group(1)}{attrs}{match.group(3)}'

        text, count = re.subn(pattern, _rewrite, text, count=1, flags=re.S)
        if count != 1:
            raise CalibrationError(f'joint {joint!r}: no <limit> found in URDF xacro')
    return text


def apply_to_text(
    text: str,
    calibration: dict[str, dict],
    *,
    margin_rad: float = 0.0,
    full_turn_motors: Iterable[str] = (),
) -> tuple[str, dict[str, tuple[float, float]], list[str]]:
    """Return rewritten YAML text, URDF limit map, and warnings."""
    config = yaml.safe_load(text) or {}
    updates, urdf_limits, warnings = plan(
        calibration, config, margin_rad=margin_rad, full_turn_motors=full_turn_motors
    )
    updates.update(sensor_updates(config, updates))
    return apply_updates(text, updates), urdf_limits, warnings


def apply_to_file(
    path: Path, calibration: dict[str, dict]
) -> tuple[str, dict[str, tuple[float, float]], list[str]]:
    """Return rewritten YAML text, URDF limit map, and warnings for a YAML file."""
    return apply_to_text(path.read_text(encoding='utf-8'), calibration)


def homing_offsets_by_actuator(calibration: dict[str, dict], config: dict) -> dict[str, int]:
    """Each actuator's ``homing_offset``, keyed by actuator id (records without one skipped)."""
    by_pin = {
        int(a['physical_pin']): str(a['id'])
        for a in config.get('actuators') or []
        if a.get('physical_pin') is not None
    }
    return {
        by_pin[int(record['id'])]: int(record['homing_offset'])
        for record in calibration.values()
        if 'homing_offset' in record and int(record['id']) in by_pin
    }


def set_calibration_block(
    text: str,
    *,
    calibrated_at: str,
    homing_offsets: dict[str, int],
    adapter_serial: str = '',
) -> str:
    """
    Add or replace the root ``calibration:`` block, leaving the rest of the text alone.

    Its presence is what ties a preset's limits to the EEPROM of one arm: activating
    the preset applies them to the URDF. ``adapter_serial`` is empty when calibrated
    outside the wizard, which has no way to know it.
    """
    block = yaml.safe_dump(
        {
            'calibration': {
                'adapter_serial': adapter_serial,
                'calibrated_at': calibrated_at,
                'homing_offsets': homing_offsets,
            }
        },
        sort_keys=False,
    )
    stripped = re.sub(r'(?m)^calibration:[^\n]*\n(?:[ \t]+[^\n]*\n|[ \t]*\n)*', '', text)
    return stripped.rstrip('\n') + '\n\n' + block


def urdf_limits_from_config(config: dict) -> dict[str, tuple[float, float]]:
    """Joint-space limits every mapped actuator's travel window implies."""
    limits: dict[str, tuple[float, float]] = {}
    for actuator in config.get('actuators') or []:
        joint = str(actuator.get('urdf_joint') or '').strip()
        if joint:
            limits[joint] = servo_window_to_joint_limits(
                float(actuator['limit_min']),
                float(actuator['limit_max']),
                float(actuator['offset']),
                float(actuator.get('direction', 1)),
                float(actuator.get('scale', 1)),
            )
    return limits


def generated_limit_mismatches(xacro_text: str, config: dict) -> list[str]:
    """Describe where a generated ros2_control xacro disagrees with this config."""
    problems: list[str] = []
    for actuator in config.get('actuators') or []:
        joint = str(actuator.get('urdf_joint') or '').strip()
        if not joint:
            continue
        section = re.search(rf'<joint name="{re.escape(joint)}">(.*?)</joint>', xacro_text, re.S)
        if section is None:
            problems.append(f'{joint}: not in the generated xacro')
            continue
        lower, upper = urdf_limits_from_config({'actuators': [actuator]})[joint]
        expected = {
            'offset': float(actuator['offset']),
            'limit_min': float(actuator['limit_min']),
            'limit_max': float(actuator['limit_max']),
            'min': lower,
            'max': upper,
        }
        for name, wanted in expected.items():
            found = re.search(rf'<param name="{name}">([^<]+)</param>', section.group(1))
            if found is None:
                problems.append(f'{joint}: no {name} param')
            elif abs(float(found.group(1)) - wanted) > LIMIT_TOLERANCE_RAD:
                problems.append(f'{joint}: {name} is {found.group(1)}, expected {wanted:g}')
    return problems

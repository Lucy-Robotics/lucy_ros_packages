# Copyright 2026 Lucy Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

"""Rewrite hardware YAML and URDF xacro text in place with a calibration plan."""

from __future__ import annotations

from pathlib import Path
import re
from typing import Iterable

import yaml

from lucy_config_generator.calibration.errors import CalibrationError
from lucy_config_generator.calibration.planning import plan
from lucy_config_generator.calibration.planning import sensor_updates


def _block_span(text: str, entry_id: str) -> tuple[int, int]:
    """Character span of the `- id: <entry_id>` list entry, up to the next one."""
    opener = re.search(rf"(?m)^\s*-\s+id:\s+{re.escape(entry_id)}\s*$", text)
    if opener is None:
        raise CalibrationError(f'no "- id: {entry_id}" entry found')
    start = opener.start()
    following = re.search(r"(?m)^\s*-\s+id:\s+\S+\s*$", text[opener.end() :])
    end = opener.end() + following.start() if following else len(text)
    return start, end


def apply_updates(text: str, updates: dict[str, dict]) -> str:
    """Rewrite the named keys inside each entry, leaving comments and order alone."""
    for entry_id, fields in updates.items():
        start, end = _block_span(text, entry_id)
        block = text[start:end]
        for key, value in fields.items():
            block, count = re.subn(rf"(?m)^(\s*{key}:).*$", rf"\g<1> {value:g}", block)
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
            rf'(<joint\s+name="{re.escape(joint)}"[^>]*>.*?' rf"<limit\b)([^>]*)(/>)"
        )

        def _rewrite(
            match: re.Match[str], *, lower: float = lo, upper: float = hi
        ) -> str:
            attrs = match.group(2)
            attrs, n_lo = re.subn(r'\blower="[^"]*"', f'lower="{lower:g}"', attrs)
            attrs, n_hi = re.subn(r'\bupper="[^"]*"', f'upper="{upper:g}"', attrs)
            if n_lo != 1 or n_hi != 1:
                raise CalibrationError(
                    f"joint {joint!r}: expected one lower and one upper on <limit>, "
                    f"found lower={n_lo} upper={n_hi}"
                )
            return f"{match.group(1)}{attrs}{match.group(3)}"

        text, count = re.subn(pattern, _rewrite, text, count=1, flags=re.S)
        if count != 1:
            raise CalibrationError(f"joint {joint!r}: no <limit> found in URDF xacro")
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
    return apply_to_text(path.read_text(encoding="utf-8"), calibration)

# Copyright 2026 Lucy Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

"""Hardware mapping strings must stay inert in the generated xacro, YAML and C."""

from __future__ import annotations

import re
from typing import Any

SAFE_STRING = re.compile(r'[A-Za-z0-9 _./:+-]*')


def check_safe_strings(node: Any, path: str = '') -> None:
    """Raise ValueError on any key or string value outside SAFE_STRING."""
    if isinstance(node, dict):
        for key, value in node.items():
            check_safe_strings(key, path)
            check_safe_strings(value, f'{path}.{key}' if path else str(key))
    elif isinstance(node, list):
        for i, item in enumerate(node):
            check_safe_strings(item, f'{path}[{i}]')
    elif isinstance(node, str) and not SAFE_STRING.fullmatch(node):
        raise ValueError(
            f'{path or "key"}: {node!r} has characters outside {SAFE_STRING.pattern}'
        )

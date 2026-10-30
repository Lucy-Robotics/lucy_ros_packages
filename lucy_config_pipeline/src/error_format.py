from __future__ import annotations

import json
import re

_BOARD_RE = re.compile(r'^board\s+([A-Za-z0-9_]+):\s+(.+)$')
_ACTUATOR_RE = re.compile(r'^actuator\s+([A-Za-z0-9_]+):\s+(.+)$')
_SENSOR_RE = re.compile(r'^sensor\s+([A-Za-z0-9_]+):\s+(.+)$')
_MISSING_ROOT_RE = re.compile(r'^missing root key:\s+([A-Za-z0-9_]+)$')
_LINE_COL_RE = re.compile(r'line\s+(\d+),\s+column\s+(\d+)')


def _pick_field_from_tail(tail: str) -> str:
    first = tail.split(' ', 1)[0]
    if '.' in first:
        return first
    if first.isidentifier():
        return first
    return 'unknown'


def format_error_line(line: str) -> str:
    text = line.strip()
    if not text:
        return json.dumps(
            {'field': 'unknown', 'field_path': ['unknown'], 'message': 'empty error line'}
        )

    m = _MISSING_ROOT_RE.match(text)
    if m:
        key = m.group(1)
        return json.dumps(
            {'field': key, 'field_path': [key], 'message': f'missing root key: {key}'}
        )

    m = _LINE_COL_RE.search(text)
    if m:
        line_no = int(m.group(1))
        col_no = int(m.group(2))
        return json.dumps(
            {
                'field': 'yaml.syntax',
                'field_path': ['yaml', 'syntax'],
                'line': line_no,
                'column': col_no,
                'message': text,
            }
        )

    m = _BOARD_RE.match(text)
    if m:
        board_id, tail = m.groups()
        field_tail = _pick_field_from_tail(tail)
        field = f'boards.{board_id}.{field_tail}'
        return json.dumps({'field': field, 'field_path': field.split('.'), 'message': tail})

    m = _ACTUATOR_RE.match(text)
    if m:
        act_id, tail = m.groups()
        field_tail = _pick_field_from_tail(tail)
        field = f'actuators.{act_id}.{field_tail}'
        return json.dumps({'field': field, 'field_path': field.split('.'), 'message': tail})

    m = _SENSOR_RE.match(text)
    if m:
        sensor_id, tail = m.groups()
        field_tail = _pick_field_from_tail(tail)
        field = f'sensors.{sensor_id}.{field_tail}'
        return json.dumps({'field': field, 'field_path': field.split('.'), 'message': tail})

    return json.dumps({'field': 'unknown', 'field_path': ['unknown'], 'message': text})


def format_error_lines(lines: list[str]) -> list[str]:
    return [format_error_line(line) for line in lines if line and line.strip()]

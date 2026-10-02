# Copyright 2025 Sentience Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

"""YAML hardware mapping validation."""

from __future__ import annotations

import re
from typing import Any

REQUIRED_ROOT = (
    'version',
    'robot_name',
    'firmware',
    'controller_manager',
    'boards',
    'actuators',
    'sensors',
)

# Optional root lists merged for URDF cross-check exclusions (synonyms allowed).
URDF_PASSIVE_LIST_KEYS = ('passive_urdf_joints', 'urdf_passive', 'urdf_passive_joints')
URDF_IGNORE_LIST_KEYS = ('ignore_urdf_joints', 'urdf_ignore', 'urdf_ignore_joints')
REQUIRED_ACTUATOR = (
    'id',
    'urdf_joint',
    'board',
    'virtual_pin',
    'physical_pin',
    'servo_type',
    'offset_rad',
    'direction',
    'scale',
    'servo_min_rad',
    'servo_max_rad',
    'servo_default_rad',
    'enabled',
)
# Rejected on robot YAML (degrees only allowed at LCP UI boundary).
FORBIDDEN_ACTUATOR_DEG_KEYS = (
    'offset_deg',
    'servo_min_deg',
    'servo_max_deg',
    'servo_default_deg',
)
REQUIRED_SENSOR = (
    'id',
    'type',
    'associated_actuator',
    'board',
    'virtual_pin',
    'physical_pin',
    'min_value',
    'max_value',
    'enabled',
)
REQUIRED_BOARD = (
    'serial_id',
    'board_class',
    'internal_servo_slots',
    'firmware_target',
    'compile_definition',
    'topic_actuators',
    'topic_sensors',
    'controller',
)
REQUIRED_CAMERA = (
    'name',
    'topic',
    'message_type',
    'external',
)

# Generated-artifact filenames. Directories are fixed by repo convention
# (``description/ros2_control/`` and ``config/``); only the basename is
# configurable through the optional ``generated_files`` YAML section so the
# generator, config pipeline, URDF include and launches all agree on names.
GENERATED_FILES_KEY = 'generated_files'
GENERATED_FILES_DEFAULTS: dict[str, str] = {
    'ros2_control_xacro': 'inmoov_ros2_control.xacro',
    'controllers_yaml': 'controllers.yaml',
}

# Firmware C template: single internal PWM stack vs internal + I2C (PCA) stack.
BOARD_CLASS_INTERNAL_ONLY = 'internal_servo_only'
BOARD_CLASS_INTERNAL_I2C_PWM = 'internal_servo_i2c_pwm'
# Smart bus servos daisy-chained on one UART, addressed by id rather than each
# wired to a pin. Firmware config for these is emitted as Rust, not C.
BOARD_CLASS_BUS_SERVO_ONLY = 'bus_servo_only'
BOARD_CLASSES = frozenset(
    {
        BOARD_CLASS_INTERNAL_ONLY,
        BOARD_CLASS_INTERNAL_I2C_PWM,
        BOARD_CLASS_BUS_SERVO_ONLY,
    }
)

# board_class → Cargo package directory under firmwares/ (relative to firmware.source_dir).
# Pimoroni Servo2040 is one firmware; board_class selects layout/capabilities in YAML.
BOARD_CLASS_TO_CRATE: dict[str, str] = {
    BOARD_CLASS_INTERNAL_ONLY: 'firmwares/rp2040_servo2040',
    BOARD_CLASS_INTERNAL_I2C_PWM: 'firmwares/rp2040_servo2040',
    BOARD_CLASS_BUS_SERVO_ONLY: 'firmwares/rp2040_bus_servo',
}

# Driver / Modbus adapter names emitted into per-board firmware YAML.
DRIVER_PWM_SERVO = 'PwmServoDriver'
DRIVER_BUS_SERVO = 'BusServoDriver'
DRIVER_PRESSURE = 'PressureSensorDriver'
ADAPTER_PWM_SERVO = 'PwmServoModbusAdapter'
ADAPTER_BUS_SERVO = 'BusServoModbusAdapter'
ADAPTER_PRESSURE = 'PressureSensorModbusAdapter'

_BOARD_ID_RE = re.compile(r'^rp2040_[a-z][a-z0-9_]*$')
_TOPIC_RE = re.compile(r'^[a-z][a-z0-9_/]*$')
_SERIAL_ID_RE = re.compile(r'^[A-Za-z0-9]*$')


def ros2_hardware_suffix(board_id: str) -> str:
    """Return the snake_case segment after the ``rp2040_`` board id prefix."""
    if not _BOARD_ID_RE.fullmatch(board_id):
        raise ValueError(
            f'board id {board_id!r} must match {_BOARD_ID_RE.pattern} '
            '(rp2040_ prefix + snake_case suffix)'
        )
    return board_id[7:]  # len("rp2040_") == 7


def derive_ros2_hardware_name(board_id: str) -> str:
    """
    Return ``<ros2_control name="…">`` for *board_id*.

    Pattern: ``LucyHardware`` + PascalCase of the segment after ``rp2040_``.
    Example: ``rp2040_left_arm`` → ``LucyHardwareLeftArm``.
    """
    suffix = ros2_hardware_suffix(board_id)
    parts = [p for p in suffix.split('_') if p]
    if not parts:
        raise ValueError(f'board id {board_id!r}: empty suffix after rp2040_')
    inner = ''.join(p[:1].upper() + p[1:].lower() if p else '' for p in parts)
    return 'LucyHardware' + inner


def derive_ros2_node_name(board_id: str) -> str:
    """
    Return the ``node_name`` hardware parameter for *board_id*.

    Pattern: the bare snake_case suffix after ``rp2040_``.

    ``LucySystemHardware`` names its POSIX shm/sem objects after this, and Darwin
    caps those at 31 chars — 14 once the leading '/' and ``.lucy_reg_header``
    are accounted for.
    """
    return ros2_hardware_suffix(board_id)


def resolve_generated_files(data: dict[str, Any]) -> dict[str, str]:
    """
    Return generated-artifact filenames, applying defaults for missing keys.

    The optional ``generated_files`` mapping carries bare filenames (no path
    separators); the on-disk directories are fixed by repo convention. Unknown
    keys are ignored so the section can grow without breaking older configs.
    """
    section = data.get(GENERATED_FILES_KEY)
    if section is None:
        return dict(GENERATED_FILES_DEFAULTS)
    if not isinstance(section, dict):
        raise ValueError(f'{GENERATED_FILES_KEY} must be a mapping')
    out = dict(GENERATED_FILES_DEFAULTS)
    for key in GENERATED_FILES_DEFAULTS:
        if key not in section:
            continue
        value = section[key]
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f'{GENERATED_FILES_KEY}.{key} must be a non-empty string')
        value = value.strip()
        if '/' in value or '\\' in value or value in ('.', '..'):
            raise ValueError(
                f'{GENERATED_FILES_KEY}.{key} must be a bare filename '
                '(no path separators); directories are fixed by convention'
            )
        out[key] = value
    return out


def _label(entity: dict[str, Any]) -> str:
    """Return entity id label used in validation errors."""
    value = entity.get('id')
    if isinstance(value, str) and value.strip():
        return value
    return '<unknown>'


def _append_missing_keys(
    errors: list[str], prefix: str, item: dict[str, Any], required: tuple[str, ...]
) -> bool:
    """Append missing-key errors and return False if any are missing."""
    ok = True
    for key in required:
        if key not in item:
            errors.append(f'{prefix}: missing {key}')
            ok = False
    return ok


def _validate_boards(boards: dict[str, Any], errors: list[str]) -> None:
    """Validate board definitions and append all discovered errors."""
    for board_id, board in boards.items():
        if not _append_missing_keys(errors, f'board {board_id}', board, REQUIRED_BOARD):
            continue

        controller = board['controller']
        if not isinstance(controller, dict):
            errors.append(f'board {board_id}: controller must be a mapping')
            continue
        if 'name' not in controller or 'type' not in controller:
            errors.append(f'board {board_id}: controller needs name and type')
        else:
            for key in ('name', 'type'):
                value = controller[key]
                if not isinstance(value, str) or not value.strip():
                    errors.append(f'board {board_id}: controller.{key} must be a non-empty string')

        board_class = board['board_class']
        if board_class not in BOARD_CLASSES:
            errors.append(
                f'board {board_id}: board_class must be one of {sorted(BOARD_CLASSES)}, '
                f'got {board_class!r}'
            )

        for key in ('firmware_target', 'compile_definition'):
            value = board[key]
            if not isinstance(value, str) or not value.strip():
                errors.append(f'board {board_id}: {key} must be a non-empty string')

        serial_id = board['serial_id']
        if not isinstance(serial_id, str):
            errors.append(f'board {board_id}: serial_id must be a string')
        elif serial_id and not _SERIAL_ID_RE.fullmatch(serial_id):
            errors.append(
                f'board {board_id}: serial_id must be empty or alphanumeric '
                '(USB serial / picotool --ser)'
            )

        try:
            slots = int(board['internal_servo_slots'])
        except Exception:
            errors.append(f'board {board_id}: internal_servo_slots must be an integer')
            slots = 0
        if slots < 1:
            errors.append(f'board {board_id}: internal_servo_slots must be >= 1')

        for key in ('topic_actuators', 'topic_sensors'):
            topic = board[key]
            if not isinstance(topic, str) or not topic:
                errors.append(f'board {board_id}: {key} must be a non-empty string')
            elif not _TOPIC_RE.fullmatch(topic):
                errors.append(
                    f'board {board_id}: {key} must match {_TOPIC_RE.pattern} (no leading slash)'
                )

        try:
            derive_ros2_hardware_name(board_id)
            derive_ros2_node_name(board_id)
        except ValueError as exc:
            errors.append(f'board {board_id}: {exc}')

        if board['topic_actuators'] != board['topic_actuators'].strip():
            errors.append(
                f'board {board_id}: topic_actuators must not have surrounding whitespace'
            )

        if 'slave_address' in board:
            try:
                slave = int(board['slave_address'])
            except Exception:
                errors.append(f'board {board_id}: slave_address must be an integer')
                slave = -1
            if slave < 1 or slave > 247:
                errors.append(
                    f'board {board_id}: slave_address must be in 1..247, '
                    f'got {board["slave_address"]!r}'
                )

        if 'firmware_crate' in board:
            crate = board['firmware_crate']
            if not isinstance(crate, str) or not crate.strip():
                errors.append(f'board {board_id}: firmware_crate must be a non-empty string')
            elif not crate.startswith('firmwares/'):
                errors.append(
                    f'board {board_id}: firmware_crate must start with firmwares/, got {crate!r}'
                )


def _validate_actuator(
    actuator: dict[str, Any],
    boards: dict[str, Any],
    errors: list[str],
    by_board: dict[str, list[dict[str, Any]]],
    has_item_errors_by_board: dict[str, bool],
) -> None:
    """Validate one actuator and append it to board bucket when valid."""
    aid = _label(actuator)
    for deg_key in FORBIDDEN_ACTUATOR_DEG_KEYS:
        if deg_key in actuator:
            errors.append(
                f'actuator {aid}: {deg_key} is forbidden; use radian fields '
                f'(servo_*_rad / offset_rad)'
            )
            return

    if not _append_missing_keys(errors, f'actuator {aid}', actuator, REQUIRED_ACTUATOR):
        return

    valid = True
    for key in ('id', 'urdf_joint', 'board'):
        if not isinstance(actuator[key], str) or not actuator[key].strip():
            errors.append(f'actuator {aid}: {key} must be a non-empty string')
            valid = False

    if not isinstance(actuator['enabled'], bool):
        errors.append(f'actuator {aid}: enabled must be a boolean')
        valid = False

    board_id = actuator['board']
    if board_id not in boards:
        errors.append(f'actuator {aid}: unknown board {board_id}')
        valid = False

    try:
        int(actuator['virtual_pin'])
    except Exception:
        errors.append(f'actuator {aid}: virtual_pin must be an integer')
        valid = False

    try:
        pin = int(actuator['physical_pin'])
    except Exception:
        errors.append(f'actuator {aid}: physical_pin must be an integer')
        valid = False
        pin = 0

    board_slots = 0
    board_class = None
    if board_id in boards:
        try:
            board_slots = int(boards[board_id]['internal_servo_slots'])
        except Exception:
            errors.append(
                f'actuator {aid}: cannot validate physical_pin because '
                f'board {board_id} internal_servo_slots is invalid'
            )
            valid = False
        board_class = boards[board_id].get('board_class')
    if valid and board_class != BOARD_CLASS_BUS_SERVO_ONLY and (pin < 1 or pin > board_slots):
        errors.append(
            f'actuator {aid}: physical_pin {pin} out of range 1..{board_slots}'
        )
        valid = False

    servo_type = str(actuator['servo_type']).strip('"')
    if servo_type not in ('180', '270', '300'):
        errors.append(f'actuator {aid}: invalid servo_type {servo_type!r}')
        valid = False

    try:
        float(actuator['offset_rad'])
    except Exception:
        errors.append(f'actuator {aid}: offset_rad must be numeric')
        valid = False

    try:
        direction = float(actuator['direction'])
    except Exception:
        errors.append(f'actuator {aid}: direction must be numeric')
        valid = False
        direction = 0.0
    if valid and direction not in (-1.0, 1.0):
        errors.append(f'actuator {aid}: direction must be -1 or 1')
        valid = False

    try:
        scale = float(actuator['scale'])
    except Exception:
        errors.append(f'actuator {aid}: scale must be numeric')
        valid = False
        scale = 1.0
    if valid and scale == 0.0:
        errors.append(f'actuator {aid}: scale must be non-zero')
        valid = False

    if valid:
        by_board.setdefault(board_id, []).append(actuator)
    elif board_id in boards:
        has_item_errors_by_board[board_id] = True


def _validate_actuator_ranges(
    by_board: dict[str, list[dict[str, Any]]],
    has_item_errors_by_board: dict[str, bool],
    errors: list[str],
) -> None:
    """Validate virtual pin contiguity and servo ranges for each board."""
    for board_id, actuators in by_board.items():
        if has_item_errors_by_board.get(board_id, False):
            continue
        virtual_pins = sorted(int(act['virtual_pin']) for act in actuators)
        if len(virtual_pins) != len(set(virtual_pins)):
            errors.append(f'board {board_id}: duplicate virtual_pin')
            continue
        if virtual_pins:
            max_pin = virtual_pins[-1]
            expected = set(range(max_pin + 1))
            missing = sorted(expected - set(virtual_pins))
            if missing:
                errors.append(
                    f'board {board_id}: missing virtual_pin indices {missing} '
                    '(no actuator mapped to these virtual_pin values)'
                )
        if virtual_pins != list(range(len(virtual_pins))):
            errors.append(
                f'board {board_id}: virtual_pin must be contiguous from 0..N-1, '
                f'got {virtual_pins}'
            )

        for actuator in actuators:
            aid = _label(actuator)
            lo = actuator['servo_min_rad']
            hi = actuator['servo_max_rad']
            default = actuator['servo_default_rad']
            try:
                lo_f = float(lo)
                hi_f = float(hi)
                default_f = float(default)
            except Exception:
                errors.append(
                    f'actuator {aid}: servo_min_rad/servo_max_rad/'
                    'servo_default_rad must be numeric'
                )
                continue

            if lo_f > hi_f:
                errors.append(
                    f'actuator {aid}: servo_min_rad {lo} must be <= servo_max_rad {hi}'
                )
            if default_f < lo_f or default_f > hi_f:
                errors.append(f'actuator {aid}: servo_default_rad out of [{lo}, {hi}]')


def _validate_sensor(
    sensor: dict[str, Any],
    boards: dict[str, Any],
    actuator_ids: set[str],
    errors: list[str],
    by_board: dict[str, list[dict[str, Any]]],
    has_item_errors_by_board: dict[str, bool],
) -> None:
    """Validate one sensor and append it to board bucket when valid."""
    sid = _label(sensor)
    if not _append_missing_keys(errors, f'sensor {sid}', sensor, REQUIRED_SENSOR):
        return

    valid = True
    for key in ('id', 'type', 'associated_actuator', 'board'):
        if not isinstance(sensor[key], str) or not sensor[key].strip():
            errors.append(f'sensor {sid}: {key} must be a non-empty string')
            valid = False

    if not isinstance(sensor['enabled'], bool):
        errors.append(f'sensor {sid}: enabled must be a boolean')
        valid = False

    board_id = sensor['board']
    if board_id not in boards:
        errors.append(f'sensor {sid}: unknown board {board_id}')
        valid = False

    associated_actuator = sensor['associated_actuator']
    if associated_actuator not in actuator_ids:
        errors.append(f'sensor {sid}: associated_actuator {associated_actuator} not found')
        valid = False

    try:
        int(sensor['virtual_pin'])
    except Exception:
        errors.append(f'sensor {sid}: virtual_pin must be an integer')
        valid = False

    min_value = sensor['min_value']
    max_value = sensor['max_value']
    min_num: float | None = None
    max_num: float | None = None

    if min_value is not None:
        try:
            min_num = float(min_value)
        except Exception:
            errors.append(f'sensor {sid}: min_value must be numeric or null')
            valid = False

    if max_value is not None:
        try:
            max_num = float(max_value)
        except Exception:
            errors.append(f'sensor {sid}: max_value must be numeric or null')
            valid = False

    if valid and min_num is not None and max_num is not None and min_num > max_num:
        errors.append(f'sensor {sid}: min_value {min_value} must be <= max_value {max_value}')
        valid = False

    if valid:
        by_board.setdefault(board_id, []).append(sensor)
    elif board_id in boards:
        has_item_errors_by_board[board_id] = True


def _validate_sensor_ranges(
    by_board: dict[str, list[dict[str, Any]]],
    has_item_errors_by_board: dict[str, bool],
    errors: list[str],
) -> None:
    """Validate sensor virtual-pin contiguity for each board."""
    for board_id, sensors in by_board.items():
        if has_item_errors_by_board.get(board_id, False):
            continue
        virtual_pins = sorted(int(sensor['virtual_pin']) for sensor in sensors)
        if len(virtual_pins) != len(set(virtual_pins)):
            errors.append(f'board {board_id}: duplicate sensor virtual_pin')
            continue
        if virtual_pins:
            max_pin = virtual_pins[-1]
            expected = set(range(max_pin + 1))
            missing = sorted(expected - set(virtual_pins))
            if missing:
                errors.append(
                    f'board {board_id}: missing sensor virtual_pin indices {missing} '
                    '(no sensor mapped to these virtual_pin values)'
                )
        if virtual_pins != list(range(len(virtual_pins))):
            errors.append(
                f'board {board_id}: sensor virtual_pin must be contiguous 0..N-1, '
                f'got {virtual_pins}'
            )


def validate_hardware_yaml(data: dict[str, Any]) -> None:
    """Raise ValueError if the mapping is structurally invalid."""
    for key in REQUIRED_ROOT:
        if key not in data:
            raise ValueError(f'missing root key: {key}')
    if data['version'] != 1:
        raise ValueError('version must be 1')

    resolve_generated_files(data)

    for passive_key in URDF_PASSIVE_LIST_KEYS + URDF_IGNORE_LIST_KEYS:
        plist = data.get(passive_key)
        if plist is None:
            continue
        if not isinstance(plist, list):
            raise ValueError(f'{passive_key} must be a list of strings')
        for i, item in enumerate(plist):
            if not isinstance(item, str) or not item.strip():
                raise ValueError(f'{passive_key}[{i}] must be a non-empty string')

    boards: dict[str, Any] = data['boards']
    if not isinstance(boards, dict) or not boards:
        raise ValueError('boards must be a non-empty map')

    errors: list[str] = []
    _validate_boards(boards, errors)

    actuators: list[dict[str, Any]] = data['actuators']
    if not isinstance(actuators, list):
        raise ValueError('actuators must be a list')
    actuators_by_board: dict[str, list[dict[str, Any]]] = {}
    actuator_item_errors_by_board: dict[str, bool] = {}
    for actuator in actuators:
        _validate_actuator(
            actuator,
            boards,
            errors,
            actuators_by_board,
            actuator_item_errors_by_board,
        )
    _validate_actuator_ranges(actuators_by_board, actuator_item_errors_by_board, errors)

    sensors: list[dict[str, Any]] = data['sensors']
    if not isinstance(sensors, list):
        raise ValueError('sensors must be a list')
    actuator_ids = {
        actuator.get('id')
        for actuator in actuators
        if isinstance(actuator.get('id'), str) and actuator.get('id', '').strip()
    }
    sensors_by_board: dict[str, list[dict[str, Any]]] = {}
    sensor_item_errors_by_board: dict[str, bool] = {}
    for sensor in sensors:
        _validate_sensor(
            sensor,
            boards,
            actuator_ids,
            errors,
            sensors_by_board,
            sensor_item_errors_by_board,
        )
    _validate_sensor_ranges(sensors_by_board, sensor_item_errors_by_board, errors)

    if errors:
        raise ValueError('\n'.join(errors))


def resolve_firmware_crate(board: dict[str, Any]) -> str:
    """Return firmwares/<crate> path for a board (override or board_class map)."""
    override = board.get('firmware_crate')
    if isinstance(override, str) and override.strip():
        return override.strip()
    board_class = board.get('board_class')
    if board_class not in BOARD_CLASS_TO_CRATE:
        raise ValueError(f'no firmware crate mapping for board_class {board_class!r}')
    return BOARD_CLASS_TO_CRATE[board_class]


def board_slave_address(board: dict[str, Any]) -> int:
    """Modbus slave address for a board (default 1)."""
    if 'slave_address' not in board:
        return 1
    return int(board['slave_address'])


def channel_for_actuator(actuator: dict[str, Any], board_class: str) -> str:
    """Named hardware channel for firmware YAML (ServoN / UART bus slot)."""
    if (
        'channel' in actuator
        and isinstance(actuator['channel'], str)
        and actuator['channel'].strip()
    ):
        return actuator['channel'].strip()
    pin = int(actuator['physical_pin'])
    if board_class == BOARD_CLASS_BUS_SERVO_ONLY:
        return f'UART0:{pin}'
    return f'Servo{pin}'


def channel_for_sensor(sensor: dict[str, Any]) -> str:
    """Named ADC channel for a pressure sensor."""
    if 'channel' in sensor and isinstance(sensor['channel'], str) and sensor['channel'].strip():
        return sensor['channel'].strip()
    pin = int(sensor['physical_pin'])
    # physical_pin 1 → ADC0 for RP2040-style boards
    return f'ADC{pin - 1}'


def driver_for_board_class(board_class: str) -> tuple[str, str]:
    """Return (driver, modbus_adapter) for actuators on this board class."""
    if board_class == BOARD_CLASS_BUS_SERVO_ONLY:
        return DRIVER_BUS_SERVO, ADAPTER_BUS_SERVO
    return DRIVER_PWM_SERVO, ADAPTER_PWM_SERVO

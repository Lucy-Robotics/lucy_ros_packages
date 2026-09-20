# Copyright 2025 Sentience Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

"""Jinja2 rendering for firmware, ros2_control, and controllers."""

from __future__ import annotations

from pathlib import Path
import shutil
import subprocess
from typing import Any
import xml.etree.ElementTree as ET

import jinja2

from lucy_config_generator.schema import BOARD_CLASS_BUS_SERVO_ONLY
from lucy_config_generator.schema import BOARD_CLASS_INTERNAL_I2C_PWM
from lucy_config_generator.schema import BOARD_CLASS_INTERNAL_ONLY
from lucy_config_generator.schema import board_slave_address
from lucy_config_generator.schema import channel_for_actuator
from lucy_config_generator.schema import channel_for_sensor
from lucy_config_generator.schema import derive_ros2_hardware_name
from lucy_config_generator.schema import derive_ros2_node_name
from lucy_config_generator.schema import driver_for_board_class
from lucy_config_generator.schema import DRIVER_PRESSURE
from lucy_config_generator.schema import resolve_firmware_crate
from lucy_config_generator.schema import resolve_generated_files
from lucy_config_generator.schema import validate_hardware_yaml

import yaml


def _templates_dir() -> Path:
    return Path(__file__).resolve().parent / 'templates'


def _jinja_env() -> jinja2.Environment:
    return jinja2.Environment(
        loader=jinja2.FileSystemLoader(str(_templates_dir())),
        autoescape=False,
        trim_blocks=True,
        lstrip_blocks=True,
    )


def load_hardware_yaml(path: Path) -> dict[str, Any]:
    with path.open(encoding='utf-8') as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError('YAML root must be a mapping')
    return data


def _xacro_argv(
    urdf_xacro: Path, base_path: Path, controller_config: Path
) -> list[str]:
    """Prefer ``ros2 run xacro xacro`` so broken distro ``xacro`` console scripts still work."""
    tail = [
        str(urdf_xacro),
        f'base_path:={base_path}',
        f'controller_config:={controller_config}',
        'use_gazebo_sim:=false',
    ]
    if shutil.which('ros2') is not None:
        return ['ros2', 'run', 'xacro', 'xacro', *tail]
    if shutil.which('xacro') is not None:
        return ['xacro', *tail]
    raise RuntimeError(
        """Need 'ros2' (for `ros2 run xacro xacro`) or `xacro` on PATH to process URDF"""
    )


def _parse_urdf_joints_xml(
    urdf_xml: str,
) -> tuple[set[str], dict[str, tuple[float, float]], set[str]]:
    """Joint names, URDF limits (rad), and mimic joint names."""
    root = ET.fromstring(urdf_xml)
    names: set[str] = set()
    mimics: set[str] = set()
    limits: dict[str, tuple[float, float]] = {}
    for j in root.findall('joint'):
        name = j.attrib.get('name')
        if not name:
            continue
        names.add(name)
        if j.find('mimic') is not None:
            mimics.add(name)
        if j.attrib.get('type') not in ('revolute', 'prismatic'):
            continue
        limit_el = j.find('limit')
        if limit_el is None:
            continue
        lo = limit_el.attrib.get('lower')
        hi = limit_el.attrib.get('upper')
        if lo is None or hi is None:
            continue
        limits[name] = (float(lo), float(hi))
    return names, limits, mimics


def _expand_urdf_xacro(
    urdf_xacro: Path,
    base_path: Path,
    controller_config: Path,
) -> str:
    if not urdf_xacro.is_file():
        raise FileNotFoundError(urdf_xacro)
    if not base_path.is_dir():
        raise FileNotFoundError(base_path)
    if not controller_config.is_file():
        raise FileNotFoundError(controller_config)
    cmd = _xacro_argv(urdf_xacro, base_path, controller_config)
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=120, check=False)
    if r.returncode != 0:
        raise RuntimeError(f'xacro failed: {r.stderr}')
    return r.stdout


def urdf_joint_names(
    urdf_xacro: Path,
    base_path: Path,
    controller_config: Path,
) -> set[str]:
    """Joint names from processed URDF (same args as thais_urdf xacro smoke)."""
    names, _, _ = _parse_urdf_joints_xml(
        _expand_urdf_xacro(urdf_xacro, base_path, controller_config)
    )
    return names


def urdf_joint_limits(
    urdf_xacro: Path,
    base_path: Path,
    controller_config: Path,
) -> dict[str, tuple[float, float]]:
    """URDF position limits (rad) keyed by joint name."""
    _, limits, _ = _parse_urdf_joints_xml(
        _expand_urdf_xacro(urdf_xacro, base_path, controller_config)
    )
    return limits


def _board_ids_in_yaml_order(data: dict[str, Any]) -> list[str]:
    """Preserve ``boards:`` key order from YAML (PyYAML + Python 3.7+ dict order)."""
    boards = data['boards']
    if not isinstance(boards, dict):
        raise ValueError('boards must be a mapping')
    return list(boards.keys())


def _actuators_for_board(
    data: dict[str, Any], board_id: str, *, enabled_only: bool
) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    for a in data['actuators']:
        if a['board'] != board_id:
            continue
        if enabled_only and not a.get('enabled', True):
            continue
        out.append(a)
    out.sort(key=lambda x: int(x['virtual_pin']))
    return out


def _sensors_for_board(data: dict[str, Any], board_id: str) -> list[dict[str, Any]]:
    out = [s for s in data['sensors'] if s['board'] == board_id]
    out.sort(key=lambda x: int(x['virtual_pin']))
    return out


def _sensors_for_board_firmware(
    data: dict[str, Any], board_id: str
) -> list[dict[str, Any]]:
    """Pressure rows only if their associated actuator is enabled (matches firmware C scope)."""
    enabled_ids = {a['id'] for a in data['actuators'] if a.get('enabled', True)}
    out = [
        s
        for s in data['sensors']
        if s['board'] == board_id and s['associated_actuator'] in enabled_ids
    ]
    out.sort(key=lambda x: int(x['virtual_pin']))
    return out


def _actuator_joint_for_ros2(
    actuator: dict[str, Any],
    urdf_limits: dict[str, tuple[float, float]],
    *,
    board_class: str,
) -> dict[str, Any]:
    """Copy actuator row and attach URDF command_interface min/max when present."""
    row = dict(actuator)
    pair = urdf_limits.get(actuator['urdf_joint'])
    if pair is not None:
        row['limit_lower_rad'] = pair[0]
        row['limit_upper_rad'] = pair[1]
    if board_class == BOARD_CLASS_BUS_SERVO_ONLY:
        row['joint_type'] = 'bus_servo'
        row['bus_id'] = int(actuator['physical_pin'])
    return row


def _ros2_control_blocks(
    data: dict[str, Any],
    board_ids: list[str],
    urdf_limits: dict[str, tuple[float, float]],
) -> list[dict[str, Any]]:
    blocks: list[dict[str, Any]] = []
    for bid in board_ids:
        bdef = data['boards'][bid]
        board_class = str(bdef['board_class'])
        joints = [
            _actuator_joint_for_ros2(a, urdf_limits, board_class=board_class)
            for a in _actuators_for_board(data, bid, enabled_only=False)
        ]
        blocks.append(
            {
                'board_id': bid,
                'hardware_name': derive_ros2_hardware_name(bid),
                'publisher_topic': bdef['topic_actuators'],
                'node_name': derive_ros2_node_name(bid),
                'joints': joints,
            }
        )
    return blocks


def _gazebo_camera_entry(camera: dict[str, Any]) -> dict[str, Any] | None:
    """One Gazebo-sim camera for bridge/republish, or None when not simulated."""
    if not isinstance(camera, dict):
        return None

    topic = camera.get('topic')
    if not isinstance(topic, str) or not topic.strip():
        return None
    topic = topic.strip()

    compressed_topic = camera.get('compressed_topic')
    if not isinstance(compressed_topic, str) or not compressed_topic.strip():
        return None
    compressed_topic = compressed_topic.strip()

    external = bool(camera.get('external'))
    if external:
        gz_topic = camera.get('sim_gz_topic')
        if not isinstance(gz_topic, str) or not gz_topic.strip():
            return None
        gz_topic = gz_topic.strip()
        link = None
    else:
        link = camera.get('link')
        if not isinstance(link, str) or not link.strip():
            return None
        gz_topic = topic
        link = link.strip()
    return {
        'name': camera['name'],
        'topic': topic,
        'compressed_topic': compressed_topic,
        'gz_topic': gz_topic,
        'message_type': camera['message_type'],
        'external': external,
        'link': link,
    }


def _gazebo_bridge_cameras(data: dict[str, Any]) -> list[dict[str, Any]]:
    """Cameras bridged in sim (robot-mounted + external world cameras)."""
    cameras: list[dict[str, Any]] = []
    if 'cameras' not in data:
        return cameras
    for camera in data['cameras']:
        entry = _gazebo_camera_entry(camera)
        if entry is not None:
            cameras.append(entry)
    return cameras


def _gazebo_xacro_cameras(data: dict[str, Any]) -> list[dict[str, Any]]:
    """Robot-mounted gz camera sensors only (excludes external/world cameras)."""
    return [c for c in _gazebo_bridge_cameras(data) if not c['external']]


def _gazebo_sensors(data: dict[str, Any]) -> list[dict[str, Any]]:
    sensors: list[dict[str, Any]] = []
    if 'sensors' not in data:
        return sensors

    tmp_dict = {}
    for sensor in data['sensors']:
        board_name = sensor['board']
        tmp_dict.setdefault(board_name, []).append({})

    for board_name, board_data in data['boards'].items():
        if board_name not in tmp_dict:
            continue
        sensors.append(
            {'topic': board_data['topic_sensors'], 'sensors': tmp_dict[board_name]}
        )
    return sensors


def _extra_joints(
    data: dict[str, Any],
    urdf_joints: set[str],
    mimic_joints: set[str] | None = None,
) -> list[str]:
    """
    Joints published at default via broadcaster, not listed on Lucy hardware blocks.

    Every actuator (enabled or not) is exported under ``ros2_control`` and trajectory
    controllers; only **non-actuator** URDF joints (passive / unmapped) need
    ``extra_joints`` so ``joint_state_broadcaster`` can publish them for TF.
    Mimic joints are driven by ``robot_state_publisher`` and must not be listed.
    """
    actuated = {a['urdf_joint'] for a in data['actuators']}
    mimics = mimic_joints or set()
    extra = sorted(urdf_joints - actuated - mimics)
    return extra


def _firmware_template_for_board_class(board_class: str) -> str:
    if board_class == BOARD_CLASS_INTERNAL_I2C_PWM:
        return 'config_internal_i2c_board.c.j2'
    if board_class == BOARD_CLASS_INTERNAL_ONLY:
        return 'config_internal_only_board.c.j2'
    raise ValueError(f'unknown board_class for firmware template: {board_class!r}')


def _firmware_pulse_limits(servo_type: str) -> tuple[int, int]:
    """Return default RP2040 PWM duty counts for ~1–2 ms pulses at 50 Hz (top=24999)."""
    # Servo type currently only documents mechanical range; pulse window is shared.
    _ = servo_type
    return 1250, 2500


def render_firmware_yaml(
    data: dict[str, Any],
    board_id: str,
    env: jinja2.Environment | None = None,
) -> str:
    """Render architecture-shaped Rust builder ``config.yaml`` for one board."""
    env = env or _jinja_env()
    board = data['boards'][board_id]
    board_class = str(board['board_class'])
    driver, _adapter = driver_for_board_class(board_class)
    actuators_raw = _actuators_for_board(data, board_id, enabled_only=True)
    actuators: list[dict[str, Any]] = []
    for a in actuators_raw:
        min_pulse, max_pulse = _firmware_pulse_limits(str(a.get('servo_type', '180')))
        actuators.append(
            {
                'id': a['id'],
                'enabled': bool(a.get('enabled', True)),
                'virtual_pin': int(a['virtual_pin']),
                'urdf_joint': a['urdf_joint'],
                'min_pulse': min_pulse,
                'max_pulse': max_pulse,
                'min_angle': float(a['servo_min_rad']),
                'max_angle': float(a['servo_max_rad']),
                'default_angle': float(a['servo_default_rad']),
                'driver': driver,
                'channel': channel_for_actuator(a, board_class),
            }
        )

    sensors_raw = _sensors_for_board_firmware(data, board_id)
    sensors: list[dict[str, Any]] = []
    for s in sensors_raw:
        if not s.get('enabled', True):
            continue
        sensors.append(
            {
                'id': s['id'],
                'enabled': True,
                'virtual_pin': int(s['virtual_pin']),
                'min_value': s.get('min_value', 0),
                'max_value': s.get('max_value', 4095),
                'driver': DRIVER_PRESSURE,
                'channel': channel_for_sensor(s),
            }
        )

    tpl = env.get_template('config_board.yaml.j2')
    return tpl.render(
        board_id=board_id,
        board='rp2040',
        board_class=board_class,
        slave_address=board_slave_address(board),
        serial_id=str(board.get('serial_id') or '').strip(),
        firmware_crate=resolve_firmware_crate(board),
        actuators=actuators,
        sensors=sensors,
    )


def render_firmware_c(
    data: dict[str, Any],
    board_id: str,
    env: jinja2.Environment | None = None,
) -> str:
    """Keep deprecated C renderer for golden-test migration; prefer render_firmware_yaml."""
    env = env or _jinja_env()
    actuators = _actuators_for_board(data, board_id, enabled_only=True)
    sensors = _sensors_for_board_firmware(data, board_id)
    board_class = data['boards'][board_id]['board_class']
    tpl_name = _firmware_template_for_board_class(board_class)
    tpl = env.get_template(tpl_name)
    return tpl.render(
        board_id=board_id,
        actuators=actuators,
        sensors=sensors,
    )


def render_ros2_control_xacro(
    data: dict[str, Any],
    board_ids: list[str],
    urdf_limits: dict[str, tuple[float, float]],
    env: jinja2.Environment | None = None,
) -> str:
    env = env or _jinja_env()
    tpl = env.get_template('ros2_control.xacro.j2')
    return tpl.render(
        robot_name=data.get('robot_name', ''),
        blocks=_ros2_control_blocks(data, board_ids, urdf_limits),
    )


def render_gazebo_xacro(
    data: dict[str, Any],
    board_ids: list[str],
    urdf_limits: dict[str, tuple[float, float]],
    env: jinja2.Environment | None = None,
) -> str:
    env = env or _jinja_env()
    tpl = env.get_template('gazebo.xacro.j2')
    return tpl.render(
        robot_name=data.get('robot_name', ''),
        blocks=_ros2_control_blocks(data, board_ids, urdf_limits),
        cameras=_gazebo_xacro_cameras(data),
        sensors=_gazebo_sensors(data),
    )


def render_gazebo_bridge(
    data: dict[str, Any],
    board_ids: list[str],
    urdf_limits: dict[str, tuple[float, float]],
    env: jinja2.Environment | None = None,
) -> str:
    env = env or _jinja_env()
    tpl = env.get_template('gazebo_bridge.yaml.j2')
    return tpl.render(
        blocks=_ros2_control_blocks(data, board_ids, urdf_limits),
        cameras=_gazebo_bridge_cameras(data),
        sensors=_gazebo_sensors(data),
    )


def render_controllers_yaml(
    data: dict[str, Any],
    board_ids: list[str],
    extra_joints: list[str],
    env: jinja2.Environment | None = None,
) -> str:
    env = env or _jinja_env()
    tpl = env.get_template('controllers.yaml.j2')
    controllers: list[dict[str, Any]] = []
    update_rate = int(data['controller_manager']['update_rate'])
    for bid in board_ids:
        ctrl = data['boards'][bid]['controller']
        joints = [
            a['urdf_joint'] for a in _actuators_for_board(data, bid, enabled_only=False)
        ]
        controllers.append(
            {
                'name': ctrl['name'],
                'type': ctrl['type'],
                'joints': joints,
            }
        )
    return tpl.render(
        update_rate=update_rate,
        controllers=controllers,
        extra_joints=extra_joints,
    )


def _resolve_board_ids(
    data: dict[str, Any], boards_filter: set[str] | None
) -> list[str]:
    board_ids = _board_ids_in_yaml_order(data)
    if boards_filter is not None:
        board_ids = [b for b in board_ids if b in boards_filter]
        if not board_ids:
            raise ValueError('no boards left after --boards filter')
    return board_ids


def generate(
    *,
    input_yaml: Path,
    urdf_xacro: Path,
    base_path: Path,
    controller_config: Path,
    output_dir: Path,
    targets: set[str],
    boards_filter: set[str] | None,
    simulation_only: bool = False,
) -> None:
    """Validate YAML and write selected artifacts into output_dir."""
    data = load_hardware_yaml(input_yaml)
    validate_hardware_yaml(data)

    if simulation_only:
        ros2_control_boards = _board_ids_in_yaml_order(data)
    else:
        ros2_control_boards = _resolve_board_ids(data, boards_filter)

    output_dir.mkdir(parents=True, exist_ok=True)
    names = resolve_generated_files(data)

    urdf_xml = _expand_urdf_xacro(urdf_xacro, base_path, controller_config)
    urdf_names, urdf_limits, urdf_mimics = _parse_urdf_joints_xml(urdf_xml)

    extra: list[str] = []
    if targets & {'controllers', 'all'}:
        extra = _extra_joints(data, urdf_names, urdf_mimics)

    env = _jinja_env()

    if targets & {'firmware', 'all'} and not simulation_only:
        fw_boards = _resolve_board_ids(data, boards_filter)
        for bid in fw_boards:
            text = render_firmware_yaml(data, bid, env)
            out = output_dir / f'config_{bid}.yaml'
            out.write_text(text, encoding='utf-8')

    if targets & {'ros2_control', 'all'}:
        xacro_out = output_dir / names['ros2_control_xacro']
        xacro_out.write_text(
            render_ros2_control_xacro(data, ros2_control_boards, urdf_limits, env),
            encoding='utf-8',
        )

    if targets & {'controllers', 'all'}:
        yaml_out = output_dir / names['controllers_yaml']
        yaml_out.write_text(
            render_controllers_yaml(data, ros2_control_boards, extra, env),
            encoding='utf-8',
        )

    if targets & {'gazebo', 'all'}:
        xacro_out = output_dir / 'gazebo.xacro'
        bridge_out = output_dir / 'gazebo_bridge.yaml'
        xacro_out.write_text(
            render_gazebo_xacro(data, ros2_control_boards, urdf_limits, env),
            encoding='utf-8',
        )
        bridge_out.write_text(
            render_gazebo_bridge(data, ros2_control_boards, urdf_limits, env),
            encoding='utf-8',
        )


def generate_from_xacro_string_for_tests(
    data: dict[str, Any],
    urdf_xml: str,
    targets: set[str],
    boards_filter: set[str] | None = None,
    simulation_only: bool = False,
) -> dict[str, str]:
    """In-process generation for unit tests (no xacro binary). Returns name->content."""
    validate_hardware_yaml(data)
    if simulation_only:
        board_ids = _board_ids_in_yaml_order(data)
        firmware_boards = board_ids
    else:
        board_ids = _resolve_board_ids(data, boards_filter)
        firmware_boards = board_ids
    urdf_names, urdf_limits, urdf_mimics = _parse_urdf_joints_xml(urdf_xml)
    extra = _extra_joints(data, urdf_names, urdf_mimics)
    names = resolve_generated_files(data)
    env = _jinja_env()
    out: dict[str, str] = {}
    if targets & {'firmware', 'all'} and not simulation_only:
        for bid in firmware_boards:
            out[f'config_{bid}.yaml'] = render_firmware_yaml(data, bid, env)
    if targets & {'ros2_control', 'all'}:
        out[names['ros2_control_xacro']] = render_ros2_control_xacro(
            data, board_ids, urdf_limits, env
        )
    if targets & {'controllers', 'all'}:
        out[names['controllers_yaml']] = render_controllers_yaml(
            data, board_ids, extra, env
        )
    return out

# Copyright 2025 Sentience Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

"""
URDF position limits in generated ros2_control (real + mock enforcement).

The generated ``inmoov_ros2_control.xacro`` is consumed by different
``<plugin>`` implementations at xacro-expansion time:

* ``gz_ros2_control/GazeboSimSystem`` (Gazebo) — receives the same ``min``/``max``
  params but stock upstream does **not** clamp in ``write()`` (we do not patch
  ``gz_ros2_control`` in this workspace).
* ``lucy_ros2_control/LucySystemHardware`` with ``publish_actuators=false``
  (RViz / mock) — clamps ``hw_commands_`` to ``min``/``max``.
* ``lucy_ros2_control/LucySystemHardware`` with SHM/Modbus path (real).

These tests assert that ``min``/``max`` are emitted for every actuated joint,
match the URDF ``<limit>``, and sit **outside** any ``<xacro:if>`` so real and
mock hardware always see the same command_interface envelope.
"""

from __future__ import annotations

from pathlib import Path
import re
import xml.etree.ElementTree as ET

from lucy_config_generator.generate import generate_from_xacro_string_for_tests

import yaml

_FIXTURES = Path(__file__).resolve().parent / 'fixtures'


def _load_mapping() -> dict:
    with (_FIXTURES / 'test_mapping.yaml').open(encoding='utf-8') as f:
        return yaml.safe_load(f)


def _fixture_urdf_xml() -> str:
    return (_FIXTURES / 'test_robot.urdf.xacro').read_text(encoding='utf-8')


def _render_ros2_control(simulation_only: bool) -> str:
    return generate_from_xacro_string_for_tests(
        _load_mapping(),
        _fixture_urdf_xml(),
        targets={'ros2_control'},
        simulation_only=simulation_only,
    )['inmoov_ros2_control.xacro']


def _parse(xacro_xml: str) -> ET.Element:
    """Parse the Jinja-rendered xacro as XML, declaring the xacro namespace."""
    # ``ros2_control.xacro.j2`` uses unprefixed ``xacro:`` element names that
    # ``xml.etree`` would otherwise reject; declare the namespace so it parses.
    decl = '<robot xmlns:xacro="http://www.ros.org/wiki/xacro"'
    if decl not in xacro_xml:
        xacro_xml = xacro_xml.replace('<robot', decl, 1)
    return ET.fromstring(xacro_xml)


def _find_joint_command_interface(joint_el: ET.Element) -> ET.Element:
    cmd = joint_el.find('command_interface')
    assert cmd is not None, f"joint {joint_el.attrib.get('name')!r} missing command_interface"
    assert cmd.attrib.get('name') == 'position'
    return cmd


def _expected_limits_from_urdf(urdf_xml: str) -> dict[str, tuple[float, float]]:
    root = ET.fromstring(urdf_xml)
    limits: dict[str, tuple[float, float]] = {}
    for joint in root.findall('joint'):
        name = joint.attrib.get('name')
        if joint.attrib.get('type') not in ('revolute', 'prismatic'):
            continue
        limit = joint.find('limit')
        if name is None or limit is None:
            continue
        lo = limit.attrib.get('lower')
        hi = limit.attrib.get('upper')
        if lo is None or hi is None:
            continue
        limits[name] = (float(lo), float(hi))
    return limits


def test_every_actuated_joint_emits_position_limits():
    """Property: every actuator row in the YAML gets ``min``/``max`` on its position command."""
    data = _load_mapping()
    xacro = _render_ros2_control(simulation_only=False)
    root = _parse(xacro)

    actuator_joints = {a['urdf_joint'] for a in data['actuators']}

    seen: set[str] = set()
    for joint_el in root.iter('joint'):
        name = joint_el.attrib.get('name')
        if name not in actuator_joints:
            continue
        seen.add(name)
        cmd = _find_joint_command_interface(joint_el)
        params = {p.attrib['name']: p.text for p in cmd.findall('param')}
        assert 'min' in params, f'joint {name!r} command_interface missing <param name="min">'
        assert 'max' in params, f'joint {name!r} command_interface missing <param name="max">'

    assert seen == actuator_joints, (
        f'actuator joints not present in generated xacro: {actuator_joints - seen}'
    )


def test_position_limits_match_urdf():
    """The emitted ``min``/``max`` numerically match the URDF ``<limit>`` (rad)."""
    urdf_xml = _fixture_urdf_xml()
    expected = _expected_limits_from_urdf(urdf_xml)
    data = _load_mapping()
    actuator_joints = {a['urdf_joint'] for a in data['actuators']}

    xacro = _render_ros2_control(simulation_only=False)
    root = _parse(xacro)

    for joint_el in root.iter('joint'):
        name = joint_el.attrib.get('name')
        if name not in actuator_joints or name not in expected:
            continue
        lo_expected, hi_expected = expected[name]
        cmd = _find_joint_command_interface(joint_el)
        params = {p.attrib['name']: p.text for p in cmd.findall('param')}
        assert float(params['min']) == lo_expected, name
        assert float(params['max']) == hi_expected, name


def test_position_limits_apply_to_every_plugin_path():
    """
    Property: ``min``/``max`` are not gated on ``use_gazebo_sim`` / ``use_mock_hardware``.

    All three runtime plugins (GazeboSimSystem, LucySystemHardware-mock,
    LucySystemHardware-real) read the same ``<command_interface>`` block, so
    the limit params must sit outside every ``xacro:if`` / ``xacro:unless``.
    """
    xacro = _render_ros2_control(simulation_only=False)

    # Strip every <xacro:if>...</xacro:if> and <xacro:unless>...</xacro:unless>
    # subtree, then re-check that limits remain. If a limit param were nested
    # inside one of those gates it would disappear from this stripped view.
    stripped = re.sub(
        r'<xacro:(if|unless)\b[^>]*>.*?</xacro:\1>',
        '',
        xacro,
        flags=re.DOTALL,
    )
    assert stripped.count('<command_interface') == xacro.count('<command_interface')
    assert stripped.count('<param name="min">') == xacro.count('<param name="min">')
    assert stripped.count('<param name="max">') == xacro.count('<param name="max">')


def test_position_limits_identical_between_simulation_and_hardware_modes():
    """Generator must emit the same ``min``/``max`` set regardless of ``simulation_only``."""
    xacro_hw = _render_ros2_control(simulation_only=False)
    xacro_sim = _render_ros2_control(simulation_only=True)

    def collect(xacro_xml: str) -> dict[str, tuple[str, str]]:
        """Map joint name to (min, max) param text from generated xacro."""
        root = _parse(xacro_xml)
        out: dict[str, tuple[str, str]] = {}
        for joint_el in root.iter('joint'):
            name = joint_el.attrib.get('name')
            cmd = joint_el.find('command_interface')
            if name is None or cmd is None:
                continue
            params = {p.attrib['name']: p.text for p in cmd.findall('param')}
            if 'min' in params and 'max' in params:
                out[name] = (params['min'], params['max'])
        return out

    assert collect(xacro_hw) == collect(xacro_sim), (
        'URDF limits must match across simulation_only and hardware modes'
    )


def test_lucy_plugin_used_for_both_real_and_mock_hardware():
    """Real and mock both use ``LucySystemHardware`` for shared URDF clamping."""
    xacro = _render_ros2_control(simulation_only=False)
    hw_plugin = 'lucy_ros2_control/LucySystemHardware'
    assert hw_plugin in xacro
    assert 'mock_components/GenericSystem' not in xacro

    # ``publish_actuators`` suppresses the optional JointState debug publisher
    # in mock mode while keeping the same plugin (and URDF clamping).
    assert '<param name="publish_actuators">false</param>' in xacro
    assert '<param name="publisher_topic">' in xacro

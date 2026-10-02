#!/usr/bin/env python3
# Copyright 2025 Sentience Robotics Team
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

"""
Single entry bringup for Lucy: web panel APIs, optional real peripherals, RViz, Gazebo.

Always starts ``web_ros_api`` (rosbridge + lucy_config_pipeline).

Invalid combinations raise at launch parse time (clear ``RuntimeError``).

Arguments:
---------
- ``real`` (default ``true``): USB webcam / RealSense peripherals. When false, those
  nodes are not constructed (``OpaqueFunction``), so e.g. Docker without camera devices
  can run ``real:=false``. Hardware boards are driven via Modbus (``lucy_modbus_bridge``).
- ``rviz`` (default ``false``): RViz2 (real/sim time set per mode). With ``gazebo:=true``,
  forwarded as ``start_rviz`` to ``inmoov_urdf/gazebo.launch.py`` (no second RViz).
- ``gazebo`` (default ``false``): GZ Sim stack from ``inmoov_urdf``; requires ``real:=false``.
- ``headless`` (default ``false``): only meaningful with ``gazebo:=true``. Runs gz-sim
  server-only with EGL rendering (``-s -r --headless-rendering``) so camera sensors
  keep producing frames without an X server. Forwarded to ``inmoov_urdf/gazebo.launch.py``.

"""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _infer_robot_source_root(robot_package: str, share_dir: str) -> Path:
    """Prefer workspace src tree (pipeline writes) over install share."""
    cwd_candidate = Path.cwd() / 'src' / robot_package
    if cwd_candidate.is_dir():
        return cwd_candidate
    share = Path(share_dir)
    for parent in share.parents:
        if parent.name == 'install':
            src_candidate = parent.parent / 'src' / robot_package
            if src_candidate.is_dir():
                return src_candidate
    return share


def _discover_robot_packages():
    """
    List installed robot-description packages (launch/control.launch.py + description/).

    Identifies URDF/robot packages via their shared
    layout, so the bringup can auto-select when only one is present.
    """
    from ament_index_python.packages import get_packages_with_prefixes

    found = []
    for pkg, prefix in get_packages_with_prefixes().items():
        share = Path(prefix) / 'share' / pkg
        if (share / 'launch' / 'control.launch.py').is_file() and (
            share / 'description'
        ).is_dir():
            found.append(pkg)
    return sorted(found)


def _default_robot_package():
    """
    Auto-pick the sole robot package; prefer inmoov_urdf when several are built.

    Returns an empty string when none is discovered, leaving ``robot_package``
    unset so ``_validate_lucy_launch`` can fail with a clear message instead of
    silently assuming a package that is not installed.
    """
    pkgs = _discover_robot_packages()
    if len(pkgs) == 1:
        return pkgs[0]
    if 'inmoov_urdf' in pkgs:
        return 'inmoov_urdf'
    if pkgs:
        return pkgs[0]
    return ''


def _resolve_robot_paths(context):
    """
    Fill urdf_path / base_path / controllers_yaml from the selected robot_package.

    Reads ``config/control.launch.yaml`` (same helper as lucy_config_pipeline) so
    ``robot_package:=so_arm101_urdf`` gets ``robot.urdf.xacro``, not a hard-coded
    InMoov path. Explicit non-empty launch overrides are left untouched.
    """
    from launch.actions import SetLaunchConfiguration

    # Installed module name is ``src`` (lucy_config_pipeline setuptools layout).
    try:
        from src.robot_paths import resolve_robot_description_paths
    except ImportError as exc:
        raise RuntimeError(
            'lucy.launch.py: cannot import src.robot_paths — is '
            'lucy_config_pipeline built and the workspace overlay sourced?'
        ) from exc

    robot_package = LaunchConfiguration('robot_package').perform(context).strip()
    if not robot_package:
        # _validate_lucy_launch already raised; nothing to resolve.
        return []
    share = get_package_share_directory(robot_package)
    robot_root = _infer_robot_source_root(robot_package, share)
    urdf, base, controllers = resolve_robot_description_paths(robot_root)

    defaults = {
        'urdf_path': str(urdf),
        # Goes into a file:// URI in the xacro, so it must be posix.
        'base_path': base.as_posix(),
        'controllers_yaml': str(controllers),
    }
    actions = []
    for key, default_value in defaults.items():
        if not LaunchConfiguration(key).perform(context).strip():
            actions.append(SetLaunchConfiguration(key, default_value))
    return actions


def _validate_lucy_launch(context):
    gz = LaunchConfiguration('gazebo').perform(context).lower()
    real = LaunchConfiguration('real').perform(context).lower()
    robot_package = LaunchConfiguration('robot_package').perform(context).strip()

    if not robot_package:
        raise RuntimeError(
            'lucy.launch.py: no robot-description package found in the workspace '
            '(expected one with launch/control.launch.py + description/, e.g. '
            'inmoov_urdf). Clone/build a robot package, or pass one '
            'explicitly with robot_package:=<pkg>.'
        )

    def _is_true(val):
        return val in ('true', '1', 'yes')

    if _is_true(gz) and _is_true(real):
        raise RuntimeError(
            'lucy.launch.py: gazebo:=true conflicts with real:=true. '
            'Use real:=false for simulation (e.g. '
            '"ros2 launch lucy_bringup lucy.launch.py gazebo:=true real:=false").'
        )
    return []


def _modbus_node_name(board_id: str) -> str:
    """
    Logical ros2_control ``node_name`` for a board.

    Must match the hardware plugin ``node_name`` parameter from the generated
    xacro (``derive_ros2_node_name``: bare suffix after ``rp2040_``, e.g.
    ``rp2040_left_arm`` → ``left_arm``). POSIX SHM/sem stems are derived from
    that same string inside the HI and ``lucy_modbus_bridge.shm``.
    """
    if board_id.startswith('rp2040_'):
        return board_id[len('rp2040_'):]
    return board_id


def _resolve_hardware_yaml(context) -> Path | None:
    """Locate active hardware YAML for Modbus bridge spawn."""
    config_dir = LaunchConfiguration('config_dir').perform(context).strip()
    if config_dir:
        candidate = Path(config_dir) / 'active.yaml'
        return candidate if candidate.is_file() else None
    robot_package = LaunchConfiguration('robot_package').perform(context).strip()
    if not robot_package:
        return None
    share = get_package_share_directory(robot_package)
    robot_root = _infer_robot_source_root(robot_package, share)
    candidate = robot_root / 'config' / 'hardware' / 'active.yaml'
    return candidate if candidate.is_file() else None


def _hi_node_names_in_robot(robot_root: Path) -> set[str]:
    """``node_name`` params from generated ros2_control xacro under ``description/``."""
    import re
    import xml.etree.ElementTree as ET

    names: set[str] = set()
    desc = robot_root / 'description'
    if not desc.is_dir():
        return names
    # Fallback for xacro that is not well-formed XML (macros / unexpanded tags).
    pattern = re.compile(
        r"""<param\s+name=["']node_name["']>\s*([^<\s]+)\s*</param>"""
    )
    for path in desc.rglob('*.xacro'):
        try:
            text = path.read_text(encoding='utf-8')
        except OSError:
            continue
        try:
            root = ET.fromstring(text)
        except ET.ParseError:
            names.update(m.group(1) for m in pattern.finditer(text))
            continue
        for el in root.iter('param'):
            if el.get('name') == 'node_name' and el.text:
                names.add(el.text.strip())
    return names


def _real_hardware_stack(context, *args, **kwargs):
    """Spawn Modbus bridges (+ optional cameras) when ``real`` is true."""
    real = LaunchConfiguration('real').perform(context).lower().strip()
    if real not in ('true', '1', 'yes'):
        return []
    out = []

    hw_yaml = _resolve_hardware_yaml(context)
    if hw_yaml is not None:
        try:
            import yaml
        except ImportError as exc:  # pragma: no cover
            raise RuntimeError('PyYAML required to spawn lucy_modbus_bridge nodes') from exc
        data = yaml.safe_load(hw_yaml.read_text(encoding='utf-8')) or {}
        boards = data.get('boards') or {}
        robot_package = LaunchConfiguration('robot_package').perform(context).strip()
        hi_names: set[str] | None = None
        if robot_package:
            share = get_package_share_directory(robot_package)
            robot_root = _infer_robot_source_root(robot_package, share)
            hi_names = _hi_node_names_in_robot(robot_root)
        for board_id, bdef in boards.items():
            if not isinstance(bdef, dict):
                continue
            serial = str(bdef.get('serial_id') or '').strip()
            if not serial:
                continue
            node_name = _modbus_node_name(board_id)
            if hi_names is not None and node_name not in hi_names:
                out.append(
                    LogInfo(
                        msg=(
                            f'lucy.launch: skip Modbus bridge for {board_id}: '
                            f'no ros2_control node_name={node_name} in robot description'
                        )
                    )
                )
                continue
            out.append(
                Node(
                    package='lucy_modbus_bridge',
                    executable='modbus_bridge_node',
                    name=f'modbus_bridge_{board_id}',
                    output='screen',
                    parameters=[
                        {
                            'node_name': node_name,
                            'serial_id': serial,
                            'slave_address': 1,
                        }
                    ],
                )
            )
            out.append(
                LogInfo(msg=f'lucy.launch: Modbus bridge for {board_id} serial={serial}')
            )

    # Cameras / RealSense remain opt-in until wiring is restored.
    return out


def generate_launch_description():
    """Generate launch description for Lucy robot system."""
    audio_sample_rate_arg = DeclareLaunchArgument(
        'audio_sample_rate',
        default_value='48000',
        description='Audio sample rate in Hz (e.g., 44100, 48000)',
    )

    audio_capture_device_arg = DeclareLaunchArgument(
        'audio_capture_device',
        default_value='-1',
        description='Audio capture device index (-1 for default)',
    )

    audio_playback_device_arg = DeclareLaunchArgument(
        'audio_playback_device',
        default_value='-1',
        description='Audio playback device index (-1 for default)',
    )

    robot_package_arg = DeclareLaunchArgument(
        'robot_package',
        default_value=_default_robot_package(),
        description=(
            'Robot package: control.launch.py + config paths + RViz config + URDF + '
            'lucy_config_pipeline. Set by the launcher robot modifier '
            '(robot_package:=…). When unset, defaults to the only installed robot '
            'package, else inmoov_urdf. Forwarded to web_ros_api / the pipeline '
            '(those launches do not pick a robot on their own).'
        ),
    )

    config_dir_arg = DeclareLaunchArgument(
        'config_dir',
        default_value='',
        description=(
            'Override hardware config directory for lucy_config_pipeline '
            '(empty = <robot_package>/config/hardware)'
        ),
    )

    real_arg = DeclareLaunchArgument(
        'real',
        default_value='false',
        description='If true: lucy_modbus_bridge per board, USB webcam, RealSense',
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='If true: RViz (or start_rviz when gazebo:=true)',
    )

    gazebo_arg = DeclareLaunchArgument(
        'gazebo',
        default_value='false',
        description='If true: <robot_package> gazebo sim (requires real:=false)',
    )

    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description=(
            'Only with gazebo:=true. Server-only gz-sim with EGL rendering '
            '(-s -r --headless-rendering); cameras still work without an X server.'
        ),
    )

    # Empty defaults: _resolve_robot_paths fills these from the selected
    # robot_package at launch time, so robot_package:=<pkg> switches the URDF,
    # base meshes and controllers together. Non-empty overrides are respected.
    urdf_path_arg = DeclareLaunchArgument(
        'urdf_path',
        default_value='',
        description=(
            'URDF/xacro entry. Empty -> path from '
            '<robot_package>/config/control.launch.yaml'
        ),
    )
    base_path_arg = DeclareLaunchArgument(
        'base_path',
        default_value='',
        description='xacro base_path. Empty -> <robot_package>/description',
    )
    urdf_path = LaunchConfiguration('urdf_path')
    base_path = LaunchConfiguration('base_path')
    controllers_yaml_arg = DeclareLaunchArgument(
        'controllers_yaml',
        default_value='',
        description=(
            'controllers.yaml path. Empty -> <robot_package>/config/controllers.yaml'
        ),
    )

    validate = OpaqueFunction(function=_validate_lucy_launch)
    resolve_robot_paths = OpaqueFunction(function=_resolve_robot_paths)

    web_ros_api_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare('lucy_bringup'),
                        'launch',
                        'web_ros_api.launch.py',
                    ]
                )
            ]
        ),
        launch_arguments=[
            ('robot_package', LaunchConfiguration('robot_package')),
            ('config_dir', LaunchConfiguration('config_dir')),
        ],
    )

    controllers_yaml = LaunchConfiguration('controllers_yaml')

    # use_mock_hardware := (not gazebo) and (not real) — RViz-only mock_components path.
    use_mock_hardware = PythonExpression(
        [
            "'true' if ('",
            LaunchConfiguration('gazebo'),
            "'.lower() not in ('true','1','yes') and '",
            LaunchConfiguration('real'),
            "'.lower() not in ('true','1','yes')) else 'false'",
        ]
    )

    # Force value_type=str so ROS 2 launch does not try to YAML-parse
    # the xacro output. The URDF starts with `<?xml ...>`, which the YAML
    # loader rejects with "Unable to parse the value of parameter robot_description".
    robot_description = ParameterValue(
        Command(
            [
                'xacro ',
                urdf_path,
                ' base_path:=',
                base_path,
                ' use_gazebo_sim:=',
                LaunchConfiguration('gazebo'),
                ' use_mock_hardware:=',
                use_mock_hardware,
                ' controller_config:=',
                controllers_yaml,
            ]
        ),
        value_type=str,
    )
    robot_description_dict = {'robot_description': robot_description}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description_dict,
            {'use_sim_time': LaunchConfiguration('gazebo')},
        ],
    )

    real_hardware = OpaqueFunction(function=_real_hardware_stack)
    use_sim_time = LaunchConfiguration('gazebo')

    rviz = GroupAction(
        condition=IfCondition(LaunchConfiguration('rviz')),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare(LaunchConfiguration('robot_package')),
                                'launch',
                                'rviz.launch.py',
                            ]
                        )
                    ]
                ),
                launch_arguments=[
                    ('use_sim_time', use_sim_time),
                ],
            ),
        ],
    )

    gazebo = GroupAction(
        condition=IfCondition(LaunchConfiguration('gazebo')),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare(LaunchConfiguration('robot_package')),
                                'launch',
                                'gazebo.launch.py',
                            ]
                        )
                    ]
                ),
                launch_arguments=[
                    ('urdf_path', LaunchConfiguration('urdf_path')),
                    ('base_path', LaunchConfiguration('base_path')),
                    ('controllers_yaml', controllers_yaml),
                    ('headless', LaunchConfiguration('headless')),
                ],
            ),
        ],
    )

    ros2_control_launch = GroupAction(
        condition=UnlessCondition(LaunchConfiguration('gazebo')),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare(LaunchConfiguration('robot_package')),
                                'launch',
                                'control.launch.py',
                            ]
                        )
                    ]
                ),
                launch_arguments=[
                    ('urdf_path', LaunchConfiguration('urdf_path')),
                    ('base_path', LaunchConfiguration('base_path')),
                    ('controllers_yaml', controllers_yaml),
                    ('use_mock_hardware', use_mock_hardware),
                ],
            ),
        ],
    )

    return LaunchDescription(
        [
            audio_sample_rate_arg,
            audio_capture_device_arg,
            audio_playback_device_arg,
            robot_package_arg,
            config_dir_arg,
            real_arg,
            rviz_arg,
            gazebo_arg,
            headless_arg,
            urdf_path_arg,
            base_path_arg,
            controllers_yaml_arg,
            validate,
            resolve_robot_paths,
            LogInfo(msg='========================================'),
            LogInfo(msg='Starting lucy_bringup lucy.launch.py'),
            LogInfo(msg='========================================'),
            web_ros_api_launch,
            real_hardware,
            ros2_control_launch,
            robot_state_publisher,
            rviz,
            gazebo,
            LogInfo(msg='========================================'),
        ]
    )

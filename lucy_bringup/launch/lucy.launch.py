#!/usr/bin/env python3
# Copyright 2026 Lucy Robotics Team
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
- ``real`` (default ``true``): host SHM consumers (e.g. ``lucy-embedded-firmware-linux`` for
  ``so_arm101_urdf``), plus optional cameras / RealSense when wired. When false, those
  nodes are not constructed (``OpaqueFunction``), so e.g. Docker without the host binary
  can run ``real:=false``.
- ``rviz`` (default ``false``): RViz2 (real/sim time set per mode). With ``gazebo:=true``,
  forwarded as ``start_rviz`` to ``inmoov_urdf/gazebo.launch.py`` (no second RViz).
- ``gazebo`` (default ``false``): GZ Sim stack from ``inmoov_urdf``; requires ``real:=false``.
- ``headless`` (default ``false``): only meaningful with ``gazebo:=true``. Runs gz-sim
  server-only with EGL rendering (``-s -r --headless-rendering``) so camera sensors
  keep producing frames without an X server. Forwarded to ``inmoov_urdf/gazebo.launch.py``.

"""

import importlib.util
import os
from pathlib import Path
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
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

def _load_host_firmware():
    path = Path(__file__).resolve().parent / 'host_firmware.py'
    spec = importlib.util.spec_from_file_location('lucy_host_firmware', path)
    if spec is None or spec.loader is None:
        raise ImportError(f'cannot load {path}')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


host_firmware = _load_host_firmware()

_LINUX_FIRMWARE_BIN = 'lucy-embedded-firmware-linux'
_MCU_BRIDGE_BIN = 'lucy-embedded-firmware-mcu-bridge'


def _resolve_workspace_firmware_binary(
    *,
    bin_name: str,
    env_var: str,
    cargo_package: str,
) -> Path:
    """Locate a release binary under ``lucy_embedded_firmware``."""
    env = os.environ.get(env_var, '').strip()
    if env:
        path = Path(env).expanduser()
        if path.is_file():
            return path.resolve()
        raise RuntimeError(f'{env_var}={env!r} is not a file')

    which = shutil.which(bin_name)
    if which:
        return Path(which).resolve()

    rel = Path('src') / 'lucy_embedded_firmware' / 'target' / 'release' / bin_name
    search_roots: list[Path] = [Path.cwd(), *Path.cwd().parents]
    try:
        share = Path(get_package_share_directory('lucy_bringup'))
        search_roots.extend([share, *share.parents])
    except Exception:
        pass
    seen: set[Path] = set()
    for root in search_roots:
        try:
            root = root.resolve()
        except OSError:
            continue
        if root in seen:
            continue
        seen.add(root)
        candidate = root / rel
        if candidate.is_file():
            return candidate.resolve()

    raise RuntimeError(
        f'{bin_name} not found. Build it with '
        f'`cargo build --release -p {cargo_package}` '
        f'(from lucy_embedded_firmware), put it on PATH, or set '
        f'{env_var} to the absolute path.'
    )


def resolve_linux_firmware_binary() -> Path:
    """Locate the host Feetech SHM consumer binary (``firmwares/linux``)."""
    return _resolve_workspace_firmware_binary(
        bin_name=_LINUX_FIRMWARE_BIN,
        env_var='LUCY_EMBEDDED_FIRMWARE_LINUX',
        cargo_package='lucy-embedded-firmware-linux',
    )


def resolve_mcu_bridge_binary() -> Path:
    """Locate ``mcu_bridge``. Same search order as the host Feetech binary."""
    return _resolve_workspace_firmware_binary(
        bin_name=_MCU_BRIDGE_BIN,
        env_var='LUCY_MCU_BRIDGE',
        cargo_package='lucy-embedded-firmware-mcu-bridge',
    )


def _boards_with_serial(robot_package: str) -> list[tuple[str, str]]:
    """``(node_name, serial)`` for boards whose ``serial_id`` is non-empty."""
    import yaml
    from lucy_config_generator.schema import derive_ros2_node_name

    share = get_package_share_directory(robot_package)
    root = _infer_robot_source_root(robot_package, share)
    active = root / 'config' / 'hardware' / 'active.yaml'
    if not active.is_file():
        return []
    data = yaml.safe_load(active.read_text(encoding='utf-8'))
    if not isinstance(data, dict):
        return []
    found: list[tuple[str, str]] = []
    for board_id, board in (data.get('boards') or {}).items():
        if not isinstance(board, dict):
            continue
        serial = str(board.get('serial_id') or '').strip()
        if not serial:
            continue
        try:
            node_name = derive_ros2_node_name(str(board_id))
        except ValueError:
            continue
        found.append((node_name, serial))
    return found


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


def _load_robot_launch_defaults(robot_root: Path) -> dict[str, str]:
    """Relative path defaults from ``config/control.launch.yaml`` when present."""
    config_path = robot_root / 'config' / 'control.launch.yaml'
    if not config_path.is_file():
        return {}
    try:
        import yaml
    except ImportError:
        return {}
    try:
        data = yaml.safe_load(config_path.read_text(encoding='utf-8')) or {}
    except (OSError, yaml.YAMLError):
        return {}
    if not isinstance(data, dict):
        return {}
    out: dict[str, str] = {}
    for key in ('urdf_path', 'base_path', 'controllers_yaml'):
        value = data.get(key)
        if isinstance(value, str) and value.strip():
            out[key] = value.strip()
    return out


def _resolve_robot_paths(context):
    """
    Fill urdf_path / base_path / controllers_yaml from the selected robot_package.

    Prefers ``config/control.launch.yaml`` in the robot package (relative paths
    resolved against the package root). Falls back to the historical
    ``description/urdf/inmoov.urdf.xacro`` layout when that file is absent.

    Explicit ``urdf_path`` / ``base_path`` / ``controllers_yaml`` overrides
    (non-empty) are left untouched.
    """
    from launch.actions import SetLaunchConfiguration

    robot_package = LaunchConfiguration('robot_package').perform(context).strip()
    if not robot_package:
        # _validate_lucy_launch already raised; nothing to resolve.
        return []
    share = get_package_share_directory(robot_package)
    robot_root = _infer_robot_source_root(robot_package, share)
    launch_defaults = _load_robot_launch_defaults(robot_root)

    def _abs(rel_or_abs: str) -> str:
        p = Path(rel_or_abs)
        if p.is_absolute():
            return str(p)
        return str((robot_root / p).resolve())

    urdf_rel = launch_defaults.get(
        'urdf_path', 'description/urdf/robot.urdf.xacro'
    )
    base_rel = launch_defaults.get('base_path', 'description')
    controllers_rel = launch_defaults.get(
        'controllers_yaml', 'config/controllers.yaml'
    )

    defaults = {
        'urdf_path': _abs(urdf_rel),
        # Goes into a file:// URI in the xacro, so it must be posix.
        'base_path': Path(_abs(base_rel)).as_posix(),
        'controllers_yaml': _abs(controllers_rel),
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


def _write_real_hardware_expectation(lines: list[str]) -> None:
    """Tell the launcher readiness probe which host consumers to wait for."""
    path = Path('/tmp/.lucy_real_hardware_expected')
    path.write_text('\n'.join(lines) + '\n', encoding='utf-8')


def _write_real_hardware_crash(message: str) -> None:
    """Surface a config problem on the Real Hardware row before any process starts."""
    Path('/tmp/.lucy_real_hardware_status').write_text('1\n', encoding='utf-8')
    Path('/tmp/.lucy_real_hardware_error').write_text(message + '\n', encoding='utf-8')


def _clear_real_hardware_expectation() -> None:
    Path('/tmp/.lucy_real_hardware_expected').unlink(missing_ok=True)


def _active_boards(robot_package: str) -> dict:
    import yaml

    share = get_package_share_directory(robot_package)
    root = _infer_robot_source_root(robot_package, share)
    active = root / 'config' / 'hardware' / 'active.yaml'
    if not active.is_file():
        return {}
    data = yaml.safe_load(active.read_text(encoding='utf-8'))
    if not isinstance(data, dict):
        return {}
    boards = data.get('boards')
    return boards if isinstance(boards, dict) else {}


def _real_hardware_stack(context, *args, **kwargs):
    """Start one host program per board in the activated hardware file."""
    real = LaunchConfiguration('real').perform(context).lower().strip()
    if real not in ('true', '1', 'yes'):
        return []
    out = []
    robot_package = LaunchConfiguration('robot_package').perform(context).strip()
    try:
        targets, problems = host_firmware.host_plan(_active_boards(robot_package))
    except Exception as exc:
        _clear_real_hardware_expectation()
        out.append(LogInfo(msg=f'lucy.launch: cannot read hardware config: {exc}'))
        return out
    if problems and not targets:
        _clear_real_hardware_expectation()
        _write_real_hardware_crash(problems[0])
        out.append(LogInfo(msg=problems[0]))
        return out
    if not targets:
        _clear_real_hardware_expectation()
        out.append(
            LogInfo(
                msg=(
                    'lucy.launch: active.yaml has no host firmware to start; '
                    'skipping real hardware'
                )
            )
        )
        return out
    linux_targets = [item for item in targets if item['firmware'] == host_firmware.LINUX]
    bridge_targets = [
        item for item in targets if item['firmware'] == host_firmware.MCU_BRIDGE
    ]
    expectation: list[str] = []
    if linux_targets:
        try:
            binary = resolve_linux_firmware_binary()
        except Exception as exc:
            _clear_real_hardware_expectation()
            out.append(LogInfo(msg=f'lucy.launch: cannot start linux firmware: {exc}'))
            return out
        for item in linux_targets:
            expectation.append(f"linux {item['node']}")
            out.append(
                LogInfo(
                    msg=(
                        f"lucy.launch: starting {_LINUX_FIRMWARE_BIN} "
                        f"node_name={item['node']} device={item['device']} ({binary})"
                    )
                )
            )
            out.append(
                ExecuteProcess(
                    cmd=[
                        str(binary),
                        item['node'],
                        item['device'],
                        str(item['vid']),
                        str(item['pid']),
                    ],
                    name='lucy_embedded_firmware_linux',
                    output='screen',
                )
            )
    if bridge_targets:
        try:
            binary = resolve_mcu_bridge_binary()
        except Exception as exc:
            _clear_real_hardware_expectation()
            out.append(LogInfo(msg=f'lucy.launch: cannot start mcu_bridge: {exc}'))
            return out
        for item in bridge_targets:
            expectation.append(f"mcu-bridge {item['node']} {item['serial']}")
            out.append(
                LogInfo(
                    msg=(
                        f"lucy.launch: mcu_bridge {item['node']} "
                        f"serial={item['serial']} device={item['device']}"
                    )
                )
            )
            out.append(
                ExecuteProcess(
                    cmd=[str(binary), item['node'], item['serial'], item['device']],
                    name=f"mcu_bridge_{item['node']}",
                    output='screen',
                )
            )
    for problem in problems:
        out.append(LogInfo(msg=problem))
    _write_real_hardware_expectation(expectation)
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
            'Robot package: control.launch.py + config paths + RViz config + URDF. '
            'Defaults to the only installed robot package when just one is present, '
            'else inmoov_urdf.'
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
        description='If true: micro-ROS agents, USB webcam, RealSense',
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

    urdf_path_arg = DeclareLaunchArgument(
        'urdf_path',
        default_value='',
        description=(
            'Top-level robot xacro. Empty -> value from '
            '<robot_package>/config/control.launch.yaml '
            '(fallback: description/urdf/robot.urdf.xacro)'
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
        condition=IfCondition(LaunchConfiguration('gazebo')),
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
            robot_state_publisher,
            # Start ros2_control (creates SHM) before mcu_bridge attaches to it.
            ros2_control_launch,
            real_hardware,
            rviz,
            gazebo,
            LogInfo(msg='========================================'),
        ]
    )

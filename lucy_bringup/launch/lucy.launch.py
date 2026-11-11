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
Single entry bringup for Lucy: web panel APIs, ros2_control stack, optional RViz / Gazebo.

Always starts ``web_ros_api`` (rosbridge + lucy_config_pipeline). Invalid argument
combinations raise a ``RuntimeError`` naming the fix before anything starts.

Arguments:
---------
- ``mode`` (default ``mock``): ``mock`` runs ros2_control on mock hardware (no robot,
  no simulator); ``sim`` runs the robot package's Gazebo stack; ``real`` drives the
  physical robot through the host firmware picked by ``bridge``.
- ``bridge`` (required with ``mode:=real``, rejected otherwise): ``linux`` drives
  USB-serial Feetech servos from the host (``lucy_embedded_firmware_linux``, one per
  ros2_control node); ``mcu_bridge`` relays to the microcontroller boards of the active
  hardware YAML (``lucy_embedded_firmware_mcu_bridge``, one per board).
  ``lucy_control_supervisor`` starts them with ``ros2 run`` and restarts them with the stack.
- ``rviz`` (default ``false``): RViz2 with the robot package's config (sim time in ``sim``).
- ``headless`` (default ``false``, ``mode:=sim`` only): server-only gz-sim with EGL
  rendering, so camera sensors keep producing frames without an X server.
- ``domain_id`` (default ``$ROS_DOMAIN_ID``, else ``0``): ROS_DOMAIN_ID (0-101) of every
  process this launch starts. Run tools beside it (rqt, ``ros2 topic``, lucy-cli) with
  the same ``ROS_DOMAIN_ID`` to see the graph. One domain per robot on a shared network.
- ``robot_package``, ``config_dir``, ``urdf_path``, ``base_path``, ``controllers_yaml``:
  robot selection and path overrides (empty = robot package defaults).

Examples::

    ros2 launch lucy_bringup lucy.launch.py rviz:=true
    ros2 launch lucy_bringup lucy.launch.py mode:=sim headless:=true
    ros2 launch lucy_bringup lucy.launch.py mode:=real bridge:=linux robot_package:=so_arm101_urdf
    ros2 launch lucy_bringup lucy.launch.py mode:=real bridge:=mcu_bridge domain_id:=3

"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.actions import SetEnvironmentVariable
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

MODES = ("mock", "sim", "real")
BRIDGES = ("linux", "mcu_bridge")
# Highest ROS_DOMAIN_ID that stays clear of ephemeral ports on every platform.
MAX_DOMAIN_ID = 101
_TRUE = ("true", "1", "yes")
_FALSE = ("false", "0", "no")


def _infer_robot_source_root(robot_package: str, share_dir: str) -> Path:
    """Prefer workspace src tree (pipeline writes) over install share."""
    cwd_candidate = Path.cwd() / "src" / robot_package
    if cwd_candidate.is_dir():
        return cwd_candidate
    share = Path(share_dir)
    for parent in share.parents:
        if parent.name == "install":
            src_candidate = parent.parent / "src" / robot_package
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
        share = Path(prefix) / "share" / pkg
        if (share / "launch" / "control.launch.py").is_file() and (
            share / "description"
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
    if "inmoov_urdf" in pkgs:
        return "inmoov_urdf"
    if pkgs:
        return pkgs[0]
    return ""


def _load_robot_launch_defaults(robot_root: Path) -> dict[str, str]:
    """Relative path defaults from ``config/control.launch.yaml`` when present."""
    config_path = robot_root / "config" / "control.launch.yaml"
    if not config_path.is_file():
        return {}
    try:
        import yaml
    except ImportError:
        return {}
    try:
        data = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    except (OSError, yaml.YAMLError):
        return {}
    if not isinstance(data, dict):
        return {}
    out: dict[str, str] = {}
    for key in ("urdf_path", "base_path", "controllers_yaml"):
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
    robot_package = LaunchConfiguration("robot_package").perform(context).strip()
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

    urdf_rel = launch_defaults.get("urdf_path", "description/urdf/robot.urdf.xacro")
    base_rel = launch_defaults.get("base_path", "description")
    controllers_rel = launch_defaults.get("controllers_yaml", "config/controllers.yaml")

    defaults = {
        "urdf_path": _abs(urdf_rel),
        # Goes into a file:// URI in the xacro, so it must be posix.
        "base_path": Path(_abs(base_rel)).as_posix(),
        "controllers_yaml": _abs(controllers_rel),
    }
    actions = []
    for key, default_value in defaults.items():
        if not LaunchConfiguration(key).perform(context).strip():
            actions.append(SetLaunchConfiguration(key, default_value))
    return actions


def _flag(context, name: str) -> bool:
    value = LaunchConfiguration(name).perform(context).strip().lower()
    if value in _TRUE:
        return True
    if value in _FALSE:
        return False
    raise RuntimeError(f"lucy.launch.py: {name}:={value!r} must be true or false.")


def launch_settings(context) -> dict:
    """
    Read and check bridge / headless / domain_id / robot_package against mode.

    ``mode`` itself is checked by its DeclareLaunchArgument choices.
    """
    mode = LaunchConfiguration("mode").perform(context)
    bridge = LaunchConfiguration("bridge").perform(context).strip().lower()
    domain_id = LaunchConfiguration("domain_id").perform(context).strip()
    robot_package = LaunchConfiguration("robot_package").perform(context).strip()

    if mode == "real" and bridge not in BRIDGES:
        raise RuntimeError(
            "lucy.launch.py: mode:=real needs bridge:=linux (USB-serial servos driven "
            "from this host) or bridge:=mcu_bridge (microcontroller boards)"
            + (f", not bridge:={bridge!r}." if bridge else ".")
        )
    if mode != "real" and bridge:
        raise RuntimeError(
            f"lucy.launch.py: bridge:={bridge} only applies to mode:=real "
            f"(got mode:={mode}). Drop bridge:= or add mode:=real."
        )
    if _flag(context, "headless") and mode != "sim":
        raise RuntimeError(
            f"lucy.launch.py: headless:=true only applies to mode:=sim (got mode:={mode})."
        )
    if not domain_id.isdigit() or int(domain_id) > MAX_DOMAIN_ID:
        raise RuntimeError(
            f"lucy.launch.py: domain_id:={domain_id!r} must be an integer "
            f"from 0 to {MAX_DOMAIN_ID}."
        )
    if not robot_package:
        raise RuntimeError(
            "lucy.launch.py: no robot-description package found in the workspace "
            "(expected one with launch/control.launch.py + description/, e.g. "
            "inmoov_urdf). Clone/build a robot package, or pass one "
            "explicitly with robot_package:=<pkg>."
        )
    return {
        "mode": mode,
        "bridge": bridge,
        "domain_id": str(int(domain_id)),
        "robot_package": robot_package,
    }


def _apply_launch_settings(context):
    """
    Validate the arguments, then derive what the rest of the launch reads.

    Internal configurations: ``use_gazebo`` / ``use_mock_hardware``. Environment, set
    before any process starts so every child inherits it: ``ROS_DOMAIN_ID``, and
    ``LUCY_BRIDGE`` for lucy_control_supervisor (it starts the bridge firmwares).
    """
    settings = launch_settings(context)
    mode = settings["mode"]
    bridge = settings["bridge"] or "none"
    return [
        SetLaunchConfiguration("use_gazebo", "true" if mode == "sim" else "false"),
        SetLaunchConfiguration(
            "use_mock_hardware", "true" if mode == "mock" else "false"
        ),
        SetEnvironmentVariable("ROS_DOMAIN_ID", settings["domain_id"]),
        SetEnvironmentVariable("LUCY_BRIDGE", settings["bridge"]),
        LogInfo(
            msg=(
                f"lucy.launch: robot_package={settings['robot_package']} mode={mode} "
                f"bridge={bridge} ROS_DOMAIN_ID={settings['domain_id']}"
            )
        ),
    ]


def generate_launch_description():
    """Generate launch description for Lucy robot system."""
    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="mock",
        choices=list(MODES),
        description=(
            "mock: mock hardware (no robot, no simulator); sim: Gazebo; "
            "real: physical robot through bridge:="
        ),
    )

    bridge_arg = DeclareLaunchArgument(
        "bridge",
        default_value="",
        description=(
            "mode:=real only (required there). linux: host drives USB-serial servos "
            "(one lucy_embedded_firmware_linux per ros2_control node); mcu_bridge: host "
            "relays to the microcontroller boards of the hardware YAML"
        ),
    )

    domain_id_arg = DeclareLaunchArgument(
        "domain_id",
        default_value=EnvironmentVariable("ROS_DOMAIN_ID", default_value="0"),
        description=(
            f"ROS_DOMAIN_ID (0-{MAX_DOMAIN_ID}) for every process of this launch. "
            "Tools run beside it need the same ROS_DOMAIN_ID"
        ),
    )

    robot_package_arg = DeclareLaunchArgument(
        "robot_package",
        default_value=_default_robot_package(),
        description=(
            "Robot package: control.launch.py + config paths + RViz config + URDF. "
            "Defaults to the only installed robot package when just one is present, "
            "else inmoov_urdf."
        ),
    )

    config_dir_arg = DeclareLaunchArgument(
        "config_dir",
        default_value="",
        description=(
            "Override hardware config directory for lucy_config_pipeline "
            "(empty = <robot_package>/config/hardware)"
        ),
    )

    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="false",
        description="If true: RViz with the robot package config (sim time in mode:=sim)",
    )

    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description=(
            "mode:=sim only. Server-only gz-sim with EGL rendering "
            "(-s -r --headless-rendering); cameras still work without an X server."
        ),
    )

    urdf_path_arg = DeclareLaunchArgument(
        "urdf_path",
        default_value="",
        description=(
            "Top-level robot xacro. Empty -> value from "
            "<robot_package>/config/control.launch.yaml "
            "(fallback: description/urdf/robot.urdf.xacro)"
        ),
    )
    base_path_arg = DeclareLaunchArgument(
        "base_path",
        default_value="",
        description="xacro base_path. Empty -> <robot_package>/description",
    )
    urdf_path = LaunchConfiguration("urdf_path")
    base_path = LaunchConfiguration("base_path")
    controllers_yaml_arg = DeclareLaunchArgument(
        "controllers_yaml",
        default_value="",
        description=(
            "controllers.yaml path. Empty -> <robot_package>/config/controllers.yaml"
        ),
    )

    apply_settings = OpaqueFunction(function=_apply_launch_settings)
    resolve_robot_paths = OpaqueFunction(function=_resolve_robot_paths)

    web_ros_api_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("lucy_bringup"),
                        "launch",
                        "web_ros_api.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments=[
            ("robot_package", LaunchConfiguration("robot_package")),
            ("config_dir", LaunchConfiguration("config_dir")),
        ],
    )

    controllers_yaml = LaunchConfiguration("controllers_yaml")

    # Set by _apply_launch_settings from mode:=.
    use_gazebo = LaunchConfiguration("use_gazebo")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                urdf_path,
                " base_path:=",
                base_path,
                " use_gazebo_sim:=",
                use_gazebo,
                " use_mock_hardware:=",
                use_mock_hardware,
                " controller_config:=",
                controllers_yaml,
            ]
        ),
        value_type=str,
    )
    robot_description_dict = {"robot_description": robot_description}

    robot_state_publisher = Node(
        condition=IfCondition(use_gazebo),
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description_dict,
            {"use_sim_time": use_gazebo},
        ],
    )

    rviz = GroupAction(
        condition=IfCondition(LaunchConfiguration("rviz")),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare(LaunchConfiguration("robot_package")),
                                "launch",
                                "rviz.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments=[
                    ("use_sim_time", use_gazebo),
                ],
            ),
        ],
    )

    gazebo = GroupAction(
        condition=IfCondition(use_gazebo),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare(LaunchConfiguration("robot_package")),
                                "launch",
                                "gazebo.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments=[
                    ("urdf_path", LaunchConfiguration("urdf_path")),
                    ("base_path", LaunchConfiguration("base_path")),
                    ("controllers_yaml", controllers_yaml),
                    ("headless", LaunchConfiguration("headless")),
                ],
            ),
        ],
    )

    ros2_control_launch = GroupAction(
        condition=UnlessCondition(use_gazebo),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare(LaunchConfiguration("robot_package")),
                                "launch",
                                "control.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments=[
                    ("urdf_path", LaunchConfiguration("urdf_path")),
                    ("base_path", LaunchConfiguration("base_path")),
                    ("controllers_yaml", controllers_yaml),
                    ("use_mock_hardware", use_mock_hardware),
                ],
            ),
        ],
    )

    return LaunchDescription(
        [
            mode_arg,
            bridge_arg,
            domain_id_arg,
            robot_package_arg,
            config_dir_arg,
            rviz_arg,
            headless_arg,
            urdf_path_arg,
            base_path_arg,
            controllers_yaml_arg,
            # First: validates, then sets ROS_DOMAIN_ID / LUCY_BRIDGE before any process.
            apply_settings,
            resolve_robot_paths,
            SetEnvironmentVariable(
                "LUCY_CONFIG_DIR", LaunchConfiguration("config_dir")
            ),
            LogInfo(msg="========================================"),
            LogInfo(msg="Starting lucy_bringup lucy.launch.py"),
            LogInfo(msg="========================================"),
            web_ros_api_launch,
            robot_state_publisher,
            ros2_control_launch,
            rviz,
            gazebo,
            LogInfo(msg="========================================"),
        ]
    )

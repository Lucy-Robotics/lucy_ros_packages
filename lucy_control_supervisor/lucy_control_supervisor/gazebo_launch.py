# Copyright 2026 Lucy Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

"""
Gazebo Harmonic sim + ros2_control (sim spawners) for a robot description package.

``headless:=true`` runs gz-sim server-only with EGL rendering
(``gz sim -s -r --headless-rendering``) so camera sensors keep producing
frames without an X server, and the ros_gz camera bridge stays functional.
"""

from __future__ import annotations

import os
from pathlib import Path
import sys

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import SetEnvironmentVariable
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from lucy_control_supervisor.controllers_spawn import controllers_to_spawn
import yaml


def _headless(context) -> bool:
    return LaunchConfiguration('headless').perform(context).lower().strip() in ('true', '1', 'yes')


def _gz_plugin_path(package: str) -> str:
    try:
        ros_lib = os.path.join(get_package_prefix('gz_ros2_control'), 'lib')
    except Exception:
        try:
            share = get_package_share_directory('gz_ros2_control')
            ros_lib = os.path.join(os.path.dirname(share), 'lib')
        except Exception:
            ros_lib = ''
    mock_sensor = os.path.join(get_package_prefix(package), 'lib', 'mock_sensor')
    paths = [os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''), ros_lib]
    if os.path.isdir(mock_sensor):
        paths.append(mock_sensor)
    return os.pathsep.join(p for p in paths if p)


def _infer_robot_source_root(package: str, pkg_share: str) -> Path | None:
    """Prefer workspace ``src/<pkg>`` (writable) over install share when present."""
    cwd_candidate = Path.cwd() / 'src' / package
    if cwd_candidate.is_dir():
        return cwd_candidate
    for p in Path(pkg_share).parents:
        if p.name == 'install':
            src_candidate = p.parent / 'src' / package
            if src_candidate.is_dir():
                return src_candidate
    return None


def _active_preset(package: str, pkg_share: str) -> dict:
    src_root = _infer_robot_source_root(package, pkg_share)
    candidates = []
    if src_root is not None:
        candidates.append(src_root / 'config' / 'hardware' / 'active.yaml')
    candidates.append(Path(pkg_share) / 'config' / 'hardware' / 'active.yaml')
    for active in candidates:
        if not active.is_file():
            continue
        try:
            data = yaml.safe_load(active.read_text(encoding='utf-8'))
        except (OSError, yaml.YAMLError):
            return {}
        return data if isinstance(data, dict) else {}
    return {}


def _active_generated_files(active: dict, ros2_control_xacro: str) -> dict[str, str]:
    out = {'ros2_control_xacro': ros2_control_xacro, 'controllers_yaml': 'controllers.yaml'}
    section = active.get('generated_files')
    if isinstance(section, dict):
        for key in out:
            value = section.get(key)
            if isinstance(value, str) and value.strip() and '/' not in value:
                out[key] = value.strip()
    return out


def _default_controllers_yaml(package: str, pkg_share: str, controllers_basename: str) -> str:
    src_root = _infer_robot_source_root(package, pkg_share)
    if src_root is not None:
        cwd_candidate = src_root / 'config' / controllers_basename
        if cwd_candidate.is_file():
            return str(cwd_candidate.resolve())
    return os.path.join(pkg_share, 'config', controllers_basename)


def _sim_camera_topics(active: dict) -> list[tuple[str, str]]:
    """LCP-facing (compressed) topics for every camera simulated in Gazebo."""
    cameras = active.get('cameras')
    if not isinstance(cameras, list):
        return []
    topics: list[tuple[str, str]] = []
    for cam in cameras:
        if not isinstance(cam, dict):
            continue
        topic = cam.get('topic')
        if not isinstance(topic, str) or not topic.strip():
            continue
        compressed_topic = cam.get('compressed_topic')
        if not isinstance(compressed_topic, str) or not compressed_topic.strip():
            continue
        if cam.get('external'):
            gz_topic = cam.get('sim_gz_topic')
            if not isinstance(gz_topic, str) or not gz_topic.strip():
                continue
        elif not cam.get('link'):
            continue
        topics.append((topic.strip(), compressed_topic.strip()))
    return topics


def _gpu_env_actions(context, *args, **kwargs):
    """Headless/Jetson EGL needs a writable runtime dir; GUI must keep the user session dir."""
    gpu_mode = os.environ.get('LUCY_GPU_MODE', '').lower()
    actions = []
    if _headless(context) or gpu_mode in ('jetson', 'nvidia'):
        actions.append(
            SetEnvironmentVariable(name='XDG_RUNTIME_DIR', value='/tmp/runtime-root')
        )
    if gpu_mode in ('jetson', 'nvidia'):
        actions.extend(
            [
                SetEnvironmentVariable(
                    name='__EGL_VENDOR_LIBRARY_FILENAMES',
                    value='/usr/share/glvnd/egl_vendor.d/10_nvidia.json',
                ),
                SetEnvironmentVariable(
                    name='__GLX_VENDOR_LIBRARY_NAME', value='nvidia'
                ),
            ]
        )
    else:
        # NixOS: Pixi OGRE needs host Mesa EGL (GLX fails on Wayland/XWayland).
        nix_mesa = '/run/opengl-driver/share/glvnd/egl_vendor.d/50_mesa.json'
        if os.path.isfile(nix_mesa):
            actions.append(
                SetEnvironmentVariable(
                    name='__EGL_VENDOR_LIBRARY_FILENAMES', value=nix_mesa
                )
            )
    if not os.environ.get('GZ_IP'):
        actions.append(SetEnvironmentVariable(name='GZ_IP', value='127.0.0.1'))
    return actions


def _camera_compressor(ros_distro: str, topic: str, compressed_topic: str) -> Node:
    safe = ''.join(c if c.isalnum() else '_' for c in compressed_topic).strip('_')
    if ros_distro == 'humble':
        return Node(
            package='image_transport',
            executable='republish',
            name='camera_compressor_' + safe,
            arguments=['raw', 'compressed'],
            remappings=[
                ('in', topic),
                ('out/compressed', compressed_topic),
            ],
            parameters=[{'use_sim_time': True}],
            output='screen',
        )
    return Node(
        package='image_transport',
        executable='republish',
        name='camera_compressor_' + safe,
        remappings=[
            ('in', topic),
            ('out/compressed', compressed_topic),
        ],
        parameters=[
            {
                'use_sim_time': True,
                'in_transport': 'raw',
                'out_transport': 'compressed',
            }
        ],
        output='screen',
    )


def _create_spawner(name: str, delay: float = 0.0):
    spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[name, '--switch-timeout', '10'],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    if delay > 0.0:
        return TimerAction(period=delay, actions=[spawner])
    return spawner


def _spawner_actions_from_yaml(context, *args, **kwargs):
    yaml_path = LaunchConfiguration('controllers_yaml').perform(context)
    names = controllers_to_spawn(Path(yaml_path))
    return [_create_spawner(name, delay=float(idx * 2)) for idx, name in enumerate(names)]


def gazebo_launch_description(
    package: str, urdf: str, ros2_control_xacro: str, entity: str
) -> LaunchDescription:
    ros_distro = os.environ.get('ROS_DISTRO', 'jazzy').lower()
    pkg_share = get_package_share_directory(package)
    default_base = os.path.join(pkg_share, 'description')
    active = _active_preset(package, pkg_share)
    generated = _active_generated_files(active, ros2_control_xacro)
    default_controllers = _default_controllers_yaml(
        package, pkg_share, generated['controllers_yaml']
    )

    base_path_arg = DeclareLaunchArgument(
        'base_path',
        default_value=default_base,
        description='Base path for xacro (mesh_dir)',
    )
    controllers_yaml_arg = DeclareLaunchArgument(
        'controllers_yaml',
        default_value=default_controllers,
        description='Absolute path to controllers.yaml (pipeline write path)',
    )
    urdf_path_arg = DeclareLaunchArgument(
        'urdf_path',
        default_value=os.path.join(default_base, 'urdf', urdf),
        description='Top-level robot xacro',
    )
    ros2_control_file_arg = DeclareLaunchArgument(
        'ros2_control_file',
        default_value=generated['ros2_control_xacro'],
        description='Generated ros2_control xacro basename (generated_files in active.yaml)',
    )
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description=(
            'If true: run gz-sim server-only with EGL rendering '
            '(-s -r --headless-rendering). Camera sensors keep producing frames '
            'without an X server, so the ros_gz camera bridge stays functional.'
        ),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'use_sim_time': True,
            'config_file': os.path.join(pkg_share, 'description', 'gazebo', 'gazebo_bridge.yaml'),
        }],
        remappings=[('/world/default/clock', '/clock')],
        output='screen',
    )

    # ros_gz_bridge can only emit raw Image; the LCP wants JPEG CompressedImage.
    # For every simulated camera (robot-mounted + external world camera), republish
    # the bridged raw topic -> the compressed topic the LCP subscribes to.
    camera_compressors = [
        _camera_compressor(ros_distro, topic, compressed_topic)
        for topic, compressed_topic in _sim_camera_topics(active)
    ]

    try:
        gz_sim_launch_path = os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
        )
    except Exception as e:
        raise RuntimeError(
            'ros_gz_sim package is required for gazebo.launch.py '
            '(install ros-jazzy-ros-gz-sim or source the workspace)'
        ) from e
    default_world = os.path.join(pkg_share, 'worlds', 'default.sdf')

    def gazebo_actions(context, *args, **kwargs):
        headless = _headless(context)
        is_darwin = sys.platform == 'darwin'
        # macOS cannot run the gz server and the Qt GUI in one process: Cocoa needs
        # the GUI on the main thread, so `gz sim` refuses and exits(-1) unless given
        # -s or -g (gazebosim/gz-sim#44, enforced in the ruby entry point). Run the
        # server with -s here and start `gz sim -g` as a second process below.
        if headless:
            gz_args = f'-s -r --headless-rendering {default_world}'
        elif is_darwin:
            gz_args = f'-s -r {default_world}'
        else:
            gz_args = f'-r {default_world}'
        actions = [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_sim_launch_path),
                launch_arguments={'gz_args': gz_args}.items(),
            )
        ]
        if is_darwin and not headless:
            # Delayed so the server's transport is advertising before the GUI connects.
            actions.append(
                TimerAction(
                    period=6.0,
                    actions=[
                        ExecuteProcess(cmd=['gz', 'sim', '-g'], name='gazebo_gui', output='screen')
                    ],
                )
            )
        return actions

    spawn_robot = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=['-name', entity, '-topic', 'robot_description', '-z', '0.0'],
                output='screen',
                parameters=[{'use_sim_time': True}],
            )
        ],
    )

    supervisor = Node(
        package='lucy_control_supervisor',
        executable='control_supervisor_node',
        name='lucy_control_supervisor',
        output='screen',
        parameters=[
            {
                'urdf_path': LaunchConfiguration('urdf_path'),
                'base_path': LaunchConfiguration('base_path'),
                'controllers_yaml': LaunchConfiguration('controllers_yaml'),
                'use_gazebo_sim': True,
                'gazebo_only': True,
                'autostart': False,
                'ros2_control_file': LaunchConfiguration('ros2_control_file'),
            }
        ],
    )

    return LaunchDescription(
        [
            base_path_arg,
            controllers_yaml_arg,
            urdf_path_arg,
            ros2_control_file_arg,
            headless_arg,
            OpaqueFunction(function=_gpu_env_actions),
            SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=pkg_share),
            SetEnvironmentVariable(
                name='GZ_SIM_SYSTEM_PLUGIN_PATH', value=_gz_plugin_path(package)
            ),
            supervisor,
            OpaqueFunction(function=_spawner_actions_from_yaml),
            OpaqueFunction(function=gazebo_actions),
            spawn_robot,
            bridge,
            *camera_compressors,
        ]
    )

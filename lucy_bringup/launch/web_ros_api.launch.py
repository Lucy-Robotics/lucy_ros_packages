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
Web control panel ROS API: rosbridge WebSocket + lucy_config_pipeline.

Exposes config services and rosbridge for the Vite panel. Composed by
``lucy.launch.py``.

rosbridge listens on ``rosbridge_port`` (default ``$PORT_ROSBRIDGE``, else 9090). When that
port is taken (e.g. a second Lucy stack on this host), the next free one up to
``rosbridge_port + PORT_SEARCH_SPAN - 1`` is used and logged, instead of rosbridge retrying
the busy port forever.

"""

import os
import socket

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

DEFAULT_ROSBRIDGE_ADDRESS = "127.0.0.1"
ROSBRIDGE_ADDRESS = LaunchConfiguration("rosbridge_address", default=DEFAULT_ROSBRIDGE_ADDRESS)
# Ports tried from rosbridge_port on: rosbridge_port .. rosbridge_port + span - 1.
PORT_SEARCH_SPAN = 20


def port_is_free(address: str, port: int) -> bool:
    """
    Tell whether rosbridge could listen on ``address:port``.

    Binds the way rosbridge (tornado's ``bind_sockets``) does: every address the host
    name resolves to, with SO_REUSEADDR except on Windows, where it would let two
    servers share the port.
    """
    try:
        infos = socket.getaddrinfo(
            address or None, port, socket.AF_UNSPEC, socket.SOCK_STREAM, 0,
            socket.AI_PASSIVE,
        )
    except socket.gaierror:
        return False
    for family, kind, proto, _name, sockaddr in infos:
        with socket.socket(family, kind, proto) as sock:
            if os.name != "nt":
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            if family == socket.AF_INET6 and hasattr(socket, "IPPROTO_IPV6"):
                sock.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_V6ONLY, 1)
            try:
                sock.bind(sockaddr)
            except OSError:
                return False
    return True


def free_rosbridge_port(address: str, first: int, span: int = PORT_SEARCH_SPAN) -> int:
    """Return the first port from ``first`` that rosbridge can listen on."""
    for port in range(first, min(first + span, 65536)):
        if port_is_free(address, port):
            return port
    raise RuntimeError(
        f"web_ros_api.launch.py: no free rosbridge port on {address} in "
        f"{first}-{first + span - 1}. Stop the other stacks, or pass rosbridge_port:=<port>."
    )


def _select_rosbridge_port(context):
    """Pick rosbridge's port before it starts; log it when the requested one is busy."""
    address = ROSBRIDGE_ADDRESS.perform(context)
    requested = LaunchConfiguration("rosbridge_port").perform(context).strip()
    if not requested.isdigit() or not 0 < int(requested) < 65536:
        raise RuntimeError(
            f"web_ros_api.launch.py: rosbridge_port:={requested!r} must be a port "
            "number (1-65535)."
        )
    first = int(requested)
    port = free_rosbridge_port(address, first)
    if port == first:
        message = f"rosbridge: listening on {address}:{port}"
    else:
        message = (
            f"rosbridge: port {first} is busy, listening on {address}:{port} instead "
            f"(clients must connect to {port})"
        )
    return [
        SetLaunchConfiguration("rosbridge_selected_port", str(port)),
        LogInfo(msg=message),
    ]


def generate_launch_description():
    robot_package_arg = DeclareLaunchArgument(
        "robot_package",
        default_value="thais_urdf",
        description=(
            "Robot package share used for hardware YAML paths "
            "in lucy_config_pipeline"
        ),
    )
    rosbridge_port_arg = DeclareLaunchArgument(
        "rosbridge_port",
        default_value=EnvironmentVariable("PORT_ROSBRIDGE", default_value="9090"),
        description=(
            "rosbridge WebSocket port. When busy, the next free port "
            f"(up to +{PORT_SEARCH_SPAN - 1}) is used and logged"
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

    # Included rather than run through `ros2 launch`: that shelled out via
    # cmd.exe and a console-script shim, and on Windows shutdown signalled only
    # the outermost of those, leaving rosbridge holding the port.
    rosbridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("rosbridge_server"),
                        "launch",
                        "rosbridge_websocket_launch.xml",
                    ]
                )
            ]
        ),
        launch_arguments=[
            ("default_call_service_timeout", "60.0"),
            ("max_message_size", "50000000"),
            ("call_services_in_new_thread", "true"),
            ("send_action_goals_in_new_thread", "true"),
            ("address", ROSBRIDGE_ADDRESS),
            # Set by _select_rosbridge_port, which runs first.
            ("port", LaunchConfiguration("rosbridge_selected_port")),
        ],
    )

    config_pipeline_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("lucy_config_pipeline"),
                        "launch",
                        "config_pipeline.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments=[
            ("robot_package", LaunchConfiguration("robot_package")),
            ("config_dir", LaunchConfiguration("config_dir")),
        ],
    )

    return LaunchDescription(
        [
            robot_package_arg,
            rosbridge_port_arg,
            config_dir_arg,
            OpaqueFunction(function=_select_rosbridge_port),
            rosbridge,
            config_pipeline_launch,
        ]
    )

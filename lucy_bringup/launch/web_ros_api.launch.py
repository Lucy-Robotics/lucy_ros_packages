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
Web control panel ROS API: rosbridge WebSocket + lucy_config_pipeline.

Exposes config services and rosbridge for the Vite panel. Composed by
``lucy.launch.py``.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_package_arg = DeclareLaunchArgument(
        'robot_package',
        default_value='thais_urdf',
        description=(
            'Robot package share used for hardware YAML paths '
            'in lucy_config_pipeline'
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

    # Included rather than run through `ros2 launch`: that shelled out via
    # cmd.exe and a console-script shim, and on Windows shutdown signalled only
    # the outermost of those, leaving rosbridge holding the port.
    rosbridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare('rosbridge_server'),
                        'launch',
                        'rosbridge_websocket_launch.xml',
                    ]
                )
            ]
        ),
        launch_arguments=[
            ('default_call_service_timeout', '60.0'),
            ('max_message_size', '50000000'),
            ('call_services_in_new_thread', 'true'),
            ('send_action_goals_in_new_thread', 'true'),
        ],
    )

    config_pipeline_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lucy_config_pipeline'),
                'launch',
                'config_pipeline.launch.py',
            ])
        ]),
        launch_arguments=[
            ('robot_package', LaunchConfiguration('robot_package')),
            ('config_dir', LaunchConfiguration('config_dir')),
        ],
    )

    hardware_discovery_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lucy_hardware_discovery'),
                'launch',
                'hardware_discovery.launch.py',
            ])
        ]),
    )

    return LaunchDescription([
        robot_package_arg,
        config_dir_arg,
        rosbridge,
        config_pipeline_launch,
        hardware_discovery_launch,
    ])

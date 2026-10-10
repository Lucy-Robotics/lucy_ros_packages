#!/usr/bin/env python3
# Copyright 2024 Lucy Robotics Team
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
Launch file for zero-copy MJPEG camera publisher.

This launch file starts the camera node with optimized settings for
NVIDIA Jetson AGX Orin.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    fps_arg = DeclareLaunchArgument(
        'fps',
        default_value='10.0',
        description='Camera frame rate (1.0-30.0 FPS)'
    )

    device_arg = DeclareLaunchArgument(
        'device',
        default_value='/dev/video0',
        description='Camera device path'
    )

    vendor_id_arg = DeclareLaunchArgument(
        'vendor_id',
        default_value='',
        description='USB vendor ID for camera identification (e.g., 0x046d)'
    )

    product_id_arg = DeclareLaunchArgument(
        'product_id',
        default_value='',
        description='USB product ID for camera identification'
    )

    serial_number_arg = DeclareLaunchArgument(
        'serial_number',
        default_value='',
        description='Camera serial number for identification'
    )

    # Camera node
    camera_node = Node(
        package='camera_ros',
        executable='camera_publisher.py',
        name='camera_publisher',
        output='screen',
        parameters=[{
            'fps': LaunchConfiguration('fps'),
            'device': LaunchConfiguration('device'),
            'vendor_id': LaunchConfiguration('vendor_id'),
            'product_id': LaunchConfiguration('product_id'),
            'serial_number': LaunchConfiguration('serial_number'),
        }],
    )

    # Camera stream controller node
    camera_controller_node = Node(
        package='camera_ros',
        executable='camera_stream_controller.py',
        name='camera_stream_controller',
        output='screen',
    )

    return LaunchDescription([
        fps_arg,
        device_arg,
        vendor_id_arg,
        product_id_arg,
        serial_number_arg,
        LogInfo(
            msg='Starting zero-copy MJPEG camera publisher with '
            'service-based streaming control'
        ),
        camera_node,
        camera_controller_node,
    ])

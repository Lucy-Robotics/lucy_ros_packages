#!/usr/bin/env python3
"""Launch lucy_hardware_discovery node."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="lucy_hardware_discovery",
                executable="hardware_discovery_node",
                name="hardware_discovery_node",
                output="screen",
                parameters=[
                    {
                        "active_yaml_path": "",
                    }
                ],
            ),
        ]
    )

#!/usr/bin/env python3
# Copyright 2026 Lucy Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

"""Start lucy_control_supervisor (RSP + ros2_control + spawners, /lucy_control/restart)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument('urdf_path', default_value=''),
            DeclareLaunchArgument('base_path', default_value=''),
            DeclareLaunchArgument('controllers_yaml', default_value=''),
            DeclareLaunchArgument('use_gazebo_sim', default_value='false'),
            DeclareLaunchArgument('use_mock_hardware', default_value='false'),
            DeclareLaunchArgument('gazebo_only', default_value='false'),
            DeclareLaunchArgument('autostart', default_value='true'),
            DeclareLaunchArgument(
                'ros2_control_file', default_value='inmoov_ros2_control.xacro'
            ),
            Node(
                package='lucy_control_supervisor',
                executable='control_supervisor_node',
                name='lucy_control_supervisor',
                output='screen',
                parameters=[
                    {
                        'urdf_path': LaunchConfiguration('urdf_path'),
                        'base_path': LaunchConfiguration('base_path'),
                        'controllers_yaml': LaunchConfiguration('controllers_yaml'),
                        'use_gazebo_sim': LaunchConfiguration('use_gazebo_sim'),
                        'use_mock_hardware': LaunchConfiguration('use_mock_hardware'),
                        'gazebo_only': LaunchConfiguration('gazebo_only'),
                        'autostart': LaunchConfiguration('autostart'),
                        'ros2_control_file': LaunchConfiguration('ros2_control_file'),
                    }
                ],
            ),
        ]
    )

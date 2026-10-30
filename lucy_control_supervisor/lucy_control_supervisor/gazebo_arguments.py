# Copyright 2026 Lucy Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

from launch.actions import DeclareLaunchArgument


def gazebo_arguments(
    base_path: str, controllers_yaml: str, urdf_path: str, ros2_control_file: str
) -> list[DeclareLaunchArgument]:
    return [
        DeclareLaunchArgument(
            "base_path",
            default_value=base_path,
            description="Base path for xacro (mesh_dir)",
        ),
        DeclareLaunchArgument(
            "controllers_yaml",
            default_value=controllers_yaml,
            description="Absolute path to controllers.yaml (pipeline write path)",
        ),
        DeclareLaunchArgument(
            "urdf_path", default_value=urdf_path, description="Top-level robot xacro"
        ),
        DeclareLaunchArgument(
            "ros2_control_file",
            default_value=ros2_control_file,
            description="Generated ros2_control xacro basename (generated_files in active.yaml)",
        ),
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="If true: run gz-sim server-only with EGL rendering",
        ),
    ]

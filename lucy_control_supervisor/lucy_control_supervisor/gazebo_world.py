# Copyright 2026 Lucy Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from lucy_control_supervisor.gazebo_environment import is_headless

MACOS_GUI_DELAY_S = 6.0


def default_world(pkg_share: str) -> str:
    return os.path.join(pkg_share, "worlds", "default.sdf")


def gz_sim_launch_path() -> str:
    try:
        share = get_package_share_directory("ros_gz_sim")
    except Exception as error:
        raise RuntimeError(
            "ros_gz_sim package is required for gazebo.launch.py "
            "(install ros-jazzy-ros-gz-sim or source the workspace)"
        ) from error
    return os.path.join(share, "launch", "gz_sim.launch.py")


def gz_args(world: str, headless: bool, macos: bool) -> str:
    if headless:
        return f"-s -r --headless-rendering {world}"
    if macos:
        return f"-s -r {world}"
    return f"-r {world}"


def gazebo_actions(context, launch_path: str, world: str):
    headless = is_headless(context)
    macos = sys.platform == "darwin"
    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_path),
            launch_arguments={"gz_args": gz_args(world, headless, macos)}.items(),
        )
    ]
    if macos and not headless:
        gui = ExecuteProcess(
            cmd=["gz", "sim", "-g"], name="gazebo_gui", output="screen"
        )
        actions.append(TimerAction(period=MACOS_GUI_DELAY_S, actions=[gui]))
    return actions

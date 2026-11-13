# Copyright 2026 Lucy Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

from pathlib import Path

from launch.actions import TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from lucy_control_supervisor.controllers_spawn import controllers_to_spawn

SPAWN_ROBOT_DELAY_S = 8.0
SPAWNER_STAGGER_S = 2.0


def controller_spawner(name: str, delay: float = 0.0):
    spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[name, "--switch-timeout", "10"],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )
    if delay > 0.0:
        return TimerAction(period=delay, actions=[spawner])
    return spawner


def controller_spawner_actions(context, *args, **kwargs):
    yaml_path = LaunchConfiguration("controllers_yaml").perform(context)
    names = controllers_to_spawn(Path(yaml_path))
    return [
        controller_spawner(name, delay=index * SPAWNER_STAGGER_S)
        for index, name in enumerate(names)
    ]


def robot_spawner(entity: str) -> TimerAction:
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", entity, "-topic", "robot_description", "-z", "0.0"],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )
    return TimerAction(period=SPAWN_ROBOT_DELAY_S, actions=[spawn])

# Copyright 2026 Lucy Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from lucy_control_supervisor.gazebo_arguments import gazebo_arguments
from lucy_control_supervisor.gazebo_cameras import camera_compressors
from lucy_control_supervisor.gazebo_environment import gpu_environment_actions
from lucy_control_supervisor.gazebo_environment import gz_plugin_path
from lucy_control_supervisor.gazebo_preset import active_preset
from lucy_control_supervisor.gazebo_preset import default_controllers_yaml
from lucy_control_supervisor.gazebo_preset import generated_files
from lucy_control_supervisor.gazebo_spawners import controller_spawner_actions
from lucy_control_supervisor.gazebo_spawners import robot_spawner
from lucy_control_supervisor.gazebo_world import default_world
from lucy_control_supervisor.gazebo_world import gazebo_actions
from lucy_control_supervisor.gazebo_world import gz_sim_launch_path


def bridge_node(pkg_share: str) -> Node:
    config = os.path.join(pkg_share, "description", "gazebo", "gazebo_bridge.yaml")
    return Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"use_sim_time": True, "config_file": config}],
        remappings=[("/world/default/clock", "/clock")],
        output="screen",
    )


def supervisor_node() -> Node:
    return Node(
        package="lucy_control_supervisor",
        executable="control_supervisor_node",
        name="lucy_control_supervisor",
        output="screen",
        parameters=[
            {
                "urdf_path": LaunchConfiguration("urdf_path"),
                "base_path": LaunchConfiguration("base_path"),
                "controllers_yaml": LaunchConfiguration("controllers_yaml"),
                "use_gazebo_sim": True,
                "gazebo_only": True,
                "autostart": False,
                "ros2_control_file": LaunchConfiguration("ros2_control_file"),
            }
        ],
    )


def gazebo_launch_description(
    package: str, urdf: str, ros2_control_xacro: str, entity: str
) -> LaunchDescription:
    pkg_share = get_package_share_directory(package)
    description_dir = os.path.join(pkg_share, "description")
    active = active_preset(package, pkg_share)
    generated = generated_files(active, ros2_control_xacro)
    arguments = gazebo_arguments(
        base_path=description_dir,
        controllers_yaml=default_controllers_yaml(
            package, pkg_share, generated["controllers_yaml"]
        ),
        urdf_path=os.path.join(description_dir, "urdf", urdf),
        ros2_control_file=generated["ros2_control_xacro"],
    )
    ros_distro = os.environ.get("ROS_DISTRO", "jazzy").lower()
    world_args = [gz_sim_launch_path(), default_world(pkg_share)]

    return LaunchDescription(
        [
            *arguments,
            OpaqueFunction(function=gpu_environment_actions),
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", pkg_share),
            SetEnvironmentVariable(
                "GZ_SIM_SYSTEM_PLUGIN_PATH", gz_plugin_path(package)
            ),
            supervisor_node(),
            OpaqueFunction(function=controller_spawner_actions),
            OpaqueFunction(function=gazebo_actions, args=world_args),
            robot_spawner(entity),
            bridge_node(pkg_share),
            *camera_compressors(ros_distro, active),
        ]
    )

# Copyright 2026 Lucy Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import SetEnvironmentVariable
from launch.actions import TimerAction
from launch_ros.actions import Node
from lucy_control_supervisor import gazebo_launch


def build(tmp_path, monkeypatch, distro="jazzy"):
    monkeypatch.chdir(tmp_path)
    monkeypatch.setenv("ROS_DISTRO", distro)
    share = tmp_path / "share" / "robot_pkg"
    (share / "config" / "hardware").mkdir(parents=True)
    (share / "config" / "hardware" / "active.yaml").write_text(
        "cameras:\n  - {topic: /cam, compressed_topic: /cam/compressed, link: head}\n"
    )
    monkeypatch.setattr(
        gazebo_launch, "get_package_share_directory", lambda _: str(share)
    )
    monkeypatch.setattr(gazebo_launch, "gz_plugin_path", lambda _: "/plugins")
    monkeypatch.setattr(gazebo_launch, "gz_sim_launch_path", lambda: "/gz_sim.py")
    return gazebo_launch.gazebo_launch_description(
        "robot_pkg", "robot.urdf.xacro", "robot_ros2_control.xacro", "robot"
    )


def test_launch_description_declares_the_five_launch_arguments(tmp_path, monkeypatch):
    entities = build(tmp_path, monkeypatch).entities
    names = [e.name for e in entities if isinstance(e, DeclareLaunchArgument)]
    assert names == [
        "base_path",
        "controllers_yaml",
        "urdf_path",
        "ros2_control_file",
        "headless",
    ]


def test_launch_description_points_gazebo_at_the_robot_package(tmp_path, monkeypatch):
    entities = build(tmp_path, monkeypatch).entities
    variables = [e for e in entities if isinstance(e, SetEnvironmentVariable)]
    assert [v.name[0].text for v in variables] == [
        "GZ_SIM_RESOURCE_PATH",
        "GZ_SIM_SYSTEM_PLUGIN_PATH",
    ]
    assert len([e for e in entities if isinstance(e, OpaqueFunction)]) == 3


def test_launch_description_adds_one_compressor_per_simulated_camera(
    tmp_path, monkeypatch
):
    entities = build(tmp_path, monkeypatch).entities
    nodes = [e for e in entities if isinstance(e, Node)]
    assert len(nodes) == 3
    assert any(isinstance(e, TimerAction) for e in entities)

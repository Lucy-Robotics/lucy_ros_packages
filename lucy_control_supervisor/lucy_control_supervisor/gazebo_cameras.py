# Copyright 2026 Lucy Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

from launch_ros.actions import Node


def _text(value) -> str | None:
    if isinstance(value, str) and value.strip():
        return value.strip()
    return None


def sim_camera_topics(active: dict) -> list[tuple[str, str]]:
    cameras = active.get("cameras")
    if not isinstance(cameras, list):
        return []
    topics = []
    for camera in cameras:
        if not isinstance(camera, dict):
            continue
        topic = _text(camera.get("topic"))
        compressed_topic = _text(camera.get("compressed_topic"))
        if topic is None or compressed_topic is None:
            continue
        if camera.get("external"):
            simulated = _text(camera.get("sim_gz_topic")) is not None
        else:
            simulated = bool(camera.get("link"))
        if simulated:
            topics.append((topic, compressed_topic))
    return topics


def camera_compressor(ros_distro: str, topic: str, compressed_topic: str) -> Node:
    suffix = "".join(c if c.isalnum() else "_" for c in compressed_topic).strip("_")
    parameters = {"use_sim_time": True}
    options = {"arguments": ["raw", "compressed"]}
    if ros_distro != "humble":
        parameters.update({"in_transport": "raw", "out_transport": "compressed"})
        options = {}
    return Node(
        package="image_transport",
        executable="republish",
        name="camera_compressor_" + suffix,
        remappings=[("in", topic), ("out/compressed", compressed_topic)],
        parameters=[parameters],
        output="screen",
        **options,
    )


def camera_compressors(ros_distro: str, active: dict) -> list[Node]:
    return [
        camera_compressor(ros_distro, topic, compressed_topic)
        for topic, compressed_topic in sim_camera_topics(active)
    ]

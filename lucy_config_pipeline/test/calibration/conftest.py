"""Fixtures for the calibration backend: a robot config, a store, a lock."""

from __future__ import annotations

from pathlib import Path

import pytest
from src.config_store import ConfigStore


def _actuator(actuator_id: str, joint: str, pin: int) -> str:
    return f"""  - id: {actuator_id}
    urdf_joint: {joint}
    board: rp2040_so_arm
    virtual_pin: {pin - 1}
    physical_pin: {pin}
    servo_type: '300'
    offset: 3.14159
    direction: 1
    scale: 1
    limit_min: 1
    limit_max: 5
    default: 3.14159
    enabled: true
"""


TEMPLATE = (
    """version: 1
robot_name: so_arm101
generated_files:
  ros2_control_xacro: so_arm101_ros2_control.xacro
  controllers_yaml: controllers.yaml
passive_urdf_joints: []
ignore_urdf_joints: []
cameras: []
firmware:
  source_dir: lucy_embedded_firmware
  build_dir: build/firmware
controller_manager:
  update_rate: 100
boards:
  rp2040_so_arm:
    serial_id: ''
    board_class: bus_servo_only
    slave_address: 1
    internal_servo_slots: 4
    firmware_target: lucy_so_arm
    compile_definition: USE_SO_ARM
    topic_actuators: actuators/so_arm
    topic_sensors: sensors/so_arm
    controller:
      name: so_arm_controller
      type: joint_trajectory_controller/JointTrajectoryController
actuators:
"""
    + _actuator("rotation", "Rotation", 1)
    + _actuator("pitch", "Pitch", 2)
    + _actuator("wrist_roll", "Wrist_Roll", 3)
    + _actuator("jaw", "Jaw", 4)
    + "sensors: []\n"
)


@pytest.fixture
def store(tmp_path: Path) -> ConfigStore:
    store = ConfigStore(tmp_path / "hardware")
    store.ensure_layout()
    store.write_named_yaml("default", TEMPLATE)
    store.activate("default")
    return store

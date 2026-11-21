"""
Generic robotics interface (LeRobot-style) for LUCY.

See this package's docs/GENERIC_INTERFACE.md for the contract.
Use RobotInterface for the abstract contract; LucyROSAdapter for the ROS-backed implementation.
"""

from .base import RobotInterface
from .ros_adapter import LucyROSAdapter
from .mock_adapter import MockSpeechAdapter

__all__ = ["RobotInterface", "LucyROSAdapter", "MockSpeechAdapter"]

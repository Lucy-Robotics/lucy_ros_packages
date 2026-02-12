"""
Generic robot interface (LeRobot-style): observation/action contract.

Any controller, program, or AI model can use this interface without depending on ROS.
See this package's docs/GENERIC_INTERFACE.md.
"""

from abc import ABC, abstractmethod
from typing import Any


class RobotInterface(ABC):
    """
    Minimal LeRobot-style interface: get_observation() and send_action() with
    typed observation/action dicts. Implement this to back the robot with ROS,
    HTTP, or a mock.
    """

    @property
    @abstractmethod
    def observation_features(self) -> dict[str, Any]:
        """Keys and types/shapes of get_observation() return value. Callable when disconnected."""
        ...

    @property
    @abstractmethod
    def action_features(self) -> dict[str, Any]:
        """Keys and types/shapes expected by send_action(). Callable when disconnected."""
        ...

    @abstractmethod
    def get_observation(self) -> dict[str, Any]:
        """Return current observation dict; keys must match observation_features."""
        ...

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        """Send command to the robot; keys must match action_features. Return action applied."""
        ...

    @property
    def is_connected(self) -> bool:
        """True if the backend is connected and get_observation/send_action are valid."""
        return True


# --- Speech-only subset (can be extended with joints, images, etc.) ---

SPEECH_OBSERVATION_FEATURES = {
    "observation.audio.mic": "np.ndarray (float32, 1D) or None",
    "observation.speech.user_transcript": "str",
}

SPEECH_ACTION_FEATURES = {
    "action.speech.reply_text": "str",
    "action.speech.reply_audio": "np.ndarray (float32, 1D) or None",
}

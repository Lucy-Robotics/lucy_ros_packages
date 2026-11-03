"""Unit tests for generic interface (base, mock adapter). No ROS required."""
import sys
from pathlib import Path

import pytest

# Import from installed package or from source (project root on PYTHONPATH)
try:
    from lucy_ros_api.generic_interface import MockSpeechAdapter
    from lucy_ros_api.generic_interface.base import (
        RobotInterface,
        SPEECH_OBSERVATION_FEATURES,
        SPEECH_ACTION_FEATURES,
    )
except ImportError:
    root = Path(__file__).resolve().parent.parent
    sys.path.insert(0, str(root))
    from lucy_ros_api.generic_interface import MockSpeechAdapter
    from lucy_ros_api.generic_interface.base import (
        RobotInterface,
        SPEECH_OBSERVATION_FEATURES,
        SPEECH_ACTION_FEATURES,
    )


def test_observation_action_features_keys():
    assert "observation.audio.mic" in SPEECH_OBSERVATION_FEATURES
    assert "observation.speech.user_transcript" in SPEECH_OBSERVATION_FEATURES
    assert "action.speech.reply_text" in SPEECH_ACTION_FEATURES
    assert "action.speech.reply_audio" in SPEECH_ACTION_FEATURES


def test_mock_adapter_observation_features():
    adapter = MockSpeechAdapter()
    assert adapter.observation_features["observation.audio.mic"]
    assert adapter.observation_features["observation.speech.user_transcript"]


def test_mock_adapter_get_observation():
    adapter = MockSpeechAdapter()
    adapter.set_transcript("hello")
    adapter.set_mic_chunk(None)
    obs = adapter.get_observation()
    assert obs["observation.speech.user_transcript"] == "hello"
    assert obs["observation.audio.mic"] is None


def test_mock_adapter_send_action():
    import numpy as np
    adapter = MockSpeechAdapter()
    adapter.send_action({
        "action.speech.reply_text": "reply",
        "action.speech.reply_audio": np.array([0.1, 0.2], dtype=np.float32),
    })
    assert adapter.get_last_reply_text() == "reply"
    audio = adapter.get_last_reply_audio()
    assert audio is not None
    assert len(audio) == 2


def test_mock_adapter_is_connected():
    adapter = MockSpeechAdapter()
    assert adapter.is_connected is True

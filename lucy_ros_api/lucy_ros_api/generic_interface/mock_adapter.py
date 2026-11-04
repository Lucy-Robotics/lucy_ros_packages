"""
Mock robot interface for testing the speech pipeline without ROS or LUCY.

Feed audio from a WAV file or buffer; capture reply_audio/reply_text to files or in memory.
"""

from __future__ import annotations

from typing import Any

import numpy as np

from .base import (
    RobotInterface,
    SPEECH_ACTION_FEATURES,
    SPEECH_OBSERVATION_FEATURES,
)


class MockSpeechAdapter(RobotInterface):
    """
    Implements the speech subset of the generic interface for testing.
    - get_observation(): returns mic and transcript you set via set_mic_chunk/set_transcript.
    - send_action(): stores reply_audio and reply_text for inspection (and optionally writes WAV).
    """

    def __init__(self):
        self._mic: np.ndarray | None = None
        self._transcript: str = ""
        self._last_reply_text: str = ""
        self._last_reply_audio: np.ndarray | None = None
        self._sample_rate = 16000  # for optional WAV write

    @property
    def observation_features(self) -> dict[str, Any]:
        return dict(SPEECH_OBSERVATION_FEATURES)

    @property
    def action_features(self) -> dict[str, Any]:
        return dict(SPEECH_ACTION_FEATURES)

    def get_observation(self) -> dict[str, Any]:
        return {
            "observation.audio.mic": self._mic.copy() if self._mic is not None else None,
            "observation.speech.user_transcript": self._transcript,
        }

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        self._last_reply_text = action.get("action.speech.reply_text") or ""
        ra = action.get("action.speech.reply_audio")
        self._last_reply_audio = np.array(ra, dtype=np.float32).copy() if ra is not None else None
        return action

    def set_mic_chunk(self, pcm: np.ndarray | None) -> None:
        self._mic = pcm.copy() if pcm is not None else None

    def set_transcript(self, text: str) -> None:
        self._transcript = text

    def get_last_reply_text(self) -> str:
        return self._last_reply_text

    def get_last_reply_audio(self) -> np.ndarray | None:
        return self._last_reply_audio

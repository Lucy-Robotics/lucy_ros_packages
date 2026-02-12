"""
ROS-backed implementation of the generic robot interface (speech subset).

Subscribes to /mic_audio, publishes to /audio, uses /say action.
Requires: ROS 2 environment sourced, rclpy, audio_common_msgs.
"""

from __future__ import annotations

import threading
from typing import Any

from .base import (
    RobotInterface,
    SPEECH_ACTION_FEATURES,
    SPEECH_OBSERVATION_FEATURES,
)

# Optional ROS imports (required only when using this adapter)
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import Header
    from audio_common_msgs.msg import AudioStamped, Audio, AudioInfo, AudioData
    from audio_common_msgs.action import TTS
    from rclpy.action import ActionClient
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False

import numpy as np

# PortAudio format constants (match audio_common / LUCY player)
PA_FLOAT32 = 1
PA_INT16 = 8


class LucyROSAdapter(RobotInterface):
    """
    Generic interface adapter that talks to LUCY via ROS 2:
    - get_observation(): latest mic chunk from /mic_audio (and optional user_transcript if provided).
    - send_action(): publish reply_audio to /audio or send reply_text to /say.
    """

    def __init__(
        self,
        mic_topic: str = "/mic_audio",
        playback_topic: str = "/audio",
        say_action: str = "/say",
        playback_rate: int = 48000,
        playback_channels: int = 1,
        playback_chunk_size: int = 1024,
        node_name: str = "lucy_generic_interface",
    ):
        if not ROS_AVAILABLE:
            raise RuntimeError("rclpy or audio_common_msgs not available; cannot use LucyROSAdapter")
        self._mic_topic = mic_topic
        self._playback_topic = playback_topic
        self._say_action = say_action
        self._playback_rate = playback_rate
        self._playback_channels = playback_channels
        self._playback_chunk_size = playback_chunk_size
        self._node_name = node_name

        self._node: Node | None = None
        self._executor: SingleThreadedExecutor | None = None
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._latest_mic: np.ndarray | None = None  # float32, 1D
        self._latest_user_transcript: str = ""
        self._connected = False

    @property
    def observation_features(self) -> dict[str, Any]:
        return dict(SPEECH_OBSERVATION_FEATURES)

    @property
    def action_features(self) -> dict[str, Any]:
        return dict(SPEECH_ACTION_FEATURES)

    @property
    def is_connected(self) -> bool:
        return self._connected and self._node is not None

    def get_observation(self) -> dict[str, Any]:
        with self._lock:
            mic = self._latest_mic.copy() if self._latest_mic is not None else None
            transcript = self._latest_user_transcript
        return {
            "observation.audio.mic": mic,
            "observation.speech.user_transcript": transcript,
        }

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        if not self._node:
            return action
        reply_audio = action.get("action.speech.reply_audio")
        reply_text = action.get("action.speech.reply_text")
        if reply_audio is not None and isinstance(reply_audio, np.ndarray):
            self._publish_audio(reply_audio)
        elif reply_text:
            self._send_say(reply_text)
        return action

    def _publish_audio(self, pcm: np.ndarray) -> None:
        """Publish float32 mono PCM to /audio in chunks."""
        if self._node is None:
            return
        pub = getattr(self._node, "_playback_pub", None)
        if pub is None:
            return
        pcm = np.asarray(pcm, dtype=np.float32)
        if pcm.ndim > 1:
            pcm = pcm.ravel()
        chunk_size = self._playback_chunk_size
        num_chunks = (len(pcm) + chunk_size - 1) // chunk_size
        for i in range(0, len(pcm), chunk_size):
            chunk = pcm[i : i + chunk_size]
            msg = self._make_audio_stamped(chunk)
            pub.publish(msg)
        if self._node.get_logger():
            self._node.get_logger().info(
                "Published %d chunks (%.2f s) to %s"
                % (num_chunks, len(pcm) / self._playback_rate, self._playback_topic)
            )

    def _make_audio_stamped(self, data: np.ndarray) -> "AudioStamped":
        data = np.asarray(data, dtype=np.float32)
        msg = AudioStamped()
        msg.header = Header()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.audio = Audio()
        msg.audio.info = AudioInfo()
        msg.audio.info.format = PA_FLOAT32
        msg.audio.info.channels = self._playback_channels
        msg.audio.info.rate = self._playback_rate
        msg.audio.info.chunk = len(data)
        msg.audio.audio_data = AudioData()
        msg.audio.audio_data.float32_data = data.tolist()
        return msg

    def _send_say(self, text: str, language: str = "en") -> None:
        if self._node is None:
            return
        client = getattr(self._node, "_say_client", None)
        if client is None:
            return
        goal_msg = TTS.Goal()
        goal_msg.text = text
        goal_msg.language = language
        goal_msg.volume = 1.0
        goal_msg.rate = 1.0
        client.send_goal_async(goal_msg)

    def _mic_callback(self, msg: "AudioStamped") -> None:
        """Convert incoming mic message to float32 1D and store."""
        info = msg.audio.info
        data = msg.audio.audio_data
        if data.float32_data:
            arr = np.array(data.float32_data, dtype=np.float32)
        elif data.int16_data:
            arr = np.array(data.int16_data, dtype=np.int16).astype(np.float32) / 32768.0
        else:
            return
        if info.channels > 1:
            arr = arr.reshape(-1, info.channels).mean(axis=1)
        with self._lock:
            self._latest_mic = arr

    def set_user_transcript(self, text: str) -> None:
        """Allow pipeline to set the last ASR transcript (optional)."""
        with self._lock:
            self._latest_user_transcript = text

    def start(self) -> None:
        """Start the ROS node and spin thread. Call after rclpy.init()."""
        if not ROS_AVAILABLE or self._node is not None:
            return
        rclpy.init()
        self._node = _LucyROSNode(
            self._mic_topic,
            self._playback_topic,
            self._say_action,
            self._mic_callback,
            node_name=self._node_name,
        )
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._connected = True
        self._thread = threading.Thread(target=self._run_executor, daemon=True)
        self._thread.start()

    def _run_executor(self) -> None:
        if self._executor:
            self._executor.spin()

    def stop(self) -> None:
        """Stop the ROS node and thread."""
        self._connected = False
        if self._executor and self._node:
            self._executor.shutdown()
            self._node.destroy_node()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        self._node = None
        self._executor = None
        self._thread = None
        try:
            rclpy.shutdown()
        except Exception:
            pass


class _LucyROSNode(Node):
    """Internal ROS node for subscription, publication, and action client."""

    def __init__(
        self,
        mic_topic: str,
        playback_topic: str,
        say_action: str,
        mic_callback,
        node_name: str = "lucy_generic_interface",
    ):
        super().__init__(node_name)
        self._mic_cb = mic_callback
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self._mic_sub = self.create_subscription(
            AudioStamped,
            mic_topic,
            self._on_mic,
            qos,
        )
        self._playback_pub = self.create_publisher(AudioStamped, playback_topic, 10)
        self._say_client = ActionClient(self, TTS, say_action)
        self.get_logger().info(
            f"Generic interface: sub {mic_topic}, pub {playback_topic}, action {say_action}"
        )

    def _on_mic(self, msg: "AudioStamped") -> None:
        self._mic_cb(msg)

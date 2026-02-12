# LUCY ROS Interface

This document defines the **ROS 2 API** of the LUCY robot system. Any external component (AI models, controllers, scripts, or other processes) that needs to interact with LUCY should use only the topics, services, and actions described here.

**Stability:** Topic and action names and message types in this document are the **contract** for external clients. Changes should be versioned and documented.

---

## 1. Overview

LUCY exposes:

- **Audio I/O** – microphone stream (input) and playback stream (output).
- **TTS** – “say this text” via an action.
- **Sensing** – cameras, RealSense (if launched).
- **Actuation** – joint state and joint trajectory controllers (ros2_control).

External clients can:

- **Subscribe** to sensor/audio streams.
- **Publish** to playback or command topics (where allowed).
- **Call** services and **send goals** to actions.

---

## 2. Audio

**Message type:** `audio_common_msgs/msg/AudioStamped`

- `header` (std_msgs/Header)
- `audio` (audio_common_msgs/Audio)
  - `audio.info`: `format`, `channels`, `rate`, `chunk`
  - `audio.audio_data`: `float32_data` or `int16_data` (and others)

**Topics:** Microphone and playback use separate topics by design.

| Topic        | Type                              | Direction | Description |
|-------------|-------------------------------------|-----------|-------------|
| `/mic_audio` | `audio_common_msgs/msg/AudioStamped` | Publisher: LUCY (audio_capturer) | Microphone stream. Default: 48 kHz, stereo, int16. |
| `/audio`     | `audio_common_msgs/msg/AudioStamped` | Subscriber: LUCY (audio_player) | Playback stream. Publish here to play audio on the robot (TTS, external clients, etc.). |

**TTS action:**

| Name   | Type                          | Description |
|--------|-------------------------------|-------------|
| `/say` | `audio_common_msgs/action/TTS` | Say a text. Goal: `text`, `language` (e.g. `"en"`, `"fr"`), `volume`, `rate`. Result: `text`; feedback: `audio` (AudioStamped). |

Example (say text):

```bash
ros2 action send_goal /say audio_common_msgs/action/TTS "{text: 'Hello', language: 'en', volume: 1.0, rate: 1.0}"
```

---

## 3. Vision (Cameras)

| Topic                            | Type                              | Description |
|----------------------------------|-----------------------------------|-------------|
| `/ext_camera/jpg`                | `sensor_msgs/msg/CompressedImage`  | External camera JPEG stream (camera_ros). |
| `/realsense/color/image_raw`     | `sensor_msgs/msg/Image`           | RealSense color image (if realsense launched). |
| `/realsense/depth/image_rect_raw`| `sensor_msgs/msg/Image`           | RealSense depth (if launched). |

Services (camera_ros): `start_streaming`, `stop_streaming`, `get_client_count` (see camera_ros package).

---

## 4. Actuation (Joints)

- **Joint state:** Published by ros2_control / joint_state_broadcaster (topic name from controller config, e.g. `/joint_states`).
- **Joint commands:** Via **joint_trajectory_controller** for left/right arm:
  - Controllers: `left_arm_controller`, `right_arm_controller`
  - Command topic: typically `/left_arm_controller/joint_trajectory` and `/right_arm_controller/joint_trajectory` (follow ros2_control conventions).

Refer to `lucy_ros2_control` and launch files for exact topic names (e.g. `publisher_topic` in hardware params, and controller namespaces).

---

## 5. Summary Table (Audio + TTS – main contract for speech AI)

| Endpoint    | Type    | Role for external client            |
|------------|---------|-------------------------------------|
| `/mic_audio` | Topic (subscribe) | Receive microphone audio for ASR.   |
| `/audio`     | Topic (publish)   | Send reply audio for playback.       |
| `/say`       | Action (client)   | Request “say this text” (optional). |

Message type for audio: `audio_common_msgs/msg/AudioStamped`. Package: `audio_common_msgs` (from audio_common repo in lucy_ws).

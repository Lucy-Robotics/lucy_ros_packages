# Generic robotics interface for LUCY

This document defines the **generic controller/AI interface** (LeRobot-style) used to interact with LUCY. Any client (controller, program, or AI model) uses only this interface; how LUCY is reached (e.g. ROS, HTTP, or a mock) is implemented by **adapters** provided by this package.

---

## 1. Standard context

There is **no single formal ISO/IEC standard** for a generic robotics interface. In practice, a **de facto standard** is emerging:

- **LeRobot (Hugging Face)** defines a **Robot interface** and **LeRobotDataset** format:
  - **Live control:** `get_observation()`, `send_action(action)` with observation/action as typed dictionaries.
  - **Data:** Parquet (state/actions) + MP4 (images) + metadata.
- **NVIDIA GR00T** and others use a **LeRobot-compatible** data schema.

The **practical standard** is: **observation and action as keyed, typed structures** (e.g. `dict[str, Any]` with defined keys and shapes), with **get_observation()** and **send_action(action)** as the runtime API.

---

## 2. Interface contract

### 2.1 Runtime API

- **`get_observation() -> dict[str, Any]`**  
  Returns the current observation. Keys and value types are defined by **observation_features**.
- **`send_action(action: dict[str, Any]) -> dict[str, Any]`**  
  Sends a command to the robot. Keys must match **action_features**; returns the action that was applied (e.g. after clamping).
- **`observation_features`** and **`action_features`**  
  Properties describing the keys and types/shapes of observations and actions (callable even when not connected).

Clients depend only on this contract. The same client code can drive LUCY or another robot that implements the same interface.

### 2.2 Speech subset

A **speech subset** of the observation/action space is defined as follows. Other subsets (e.g. joints, cameras) can be added later with additional keys.

**Observation:**

| Key | Type | Description |
|-----|------|-------------|
| `observation.audio.mic` | `np.ndarray` (float32, 1D) or `None` | Latest microphone audio chunk (e.g. 16 kHz mono), or `None` if not available. |
| `observation.speech.user_transcript` | `str` | Last user utterance transcript (optional; may be set by the adapter or left for the client to fill from ASR). |

**Action:**

| Key | Type | Description |
|-----|------|-------------|
| `action.speech.reply_text` | `str` | Text to be spoken (TTS). |
| `action.speech.reply_audio` | `np.ndarray` (float32, 1D) or `None` | Raw PCM to play on the robot; if present, typically takes precedence over `reply_text` for playback. |

---

## 3. Usage

1. Obtain an implementation of the interface (e.g. from this package: `RobotInterface`, `LucyROSAdapter`, `MockSpeechAdapter`).
2. In a loop or event-driven flow:
   - Call **`get_observation()`** to read the current observation (e.g. mic chunk, transcript).
   - Compute your response (e.g. ASR → LLM → TTS, or any other logic).
   - Call **`send_action(action)`** with a dict containing the appropriate action keys (e.g. `action.speech.reply_audio` or `action.speech.reply_text`).

Clients **must not** depend on transport or middleware (e.g. ROS). They depend only on the interface (observation/action dicts and the two methods above).

---

## 4. Adapters (this package)

This package provides:

- **`RobotInterface`** – Abstract base defining the contract.
- **`LucyROSAdapter`** – Implementation that connects to LUCY via the documented ROS interface (see INTERFACE.md): subscribes to `/mic_audio`, publishes to `/audio`, calls `/say`. Use when the client runs in an environment where the LUCY ROS graph is available.
- **`MockSpeechAdapter`** – Implementation for testing: observation is set programmatically; actions are stored for inspection. No robot or ROS required.

Import from the Python package:

```python
from lucy_ros_api.generic_interface import RobotInterface, LucyROSAdapter, MockSpeechAdapter
```

The ROS adapter is the only component that uses ROS; it translates between the generic interface and LUCY’s ROS topics/actions. Clients that use `RobotInterface` (or a concrete adapter) have no direct ROS dependency.

---

## 5. References

- LeRobot: [Hugging Face LeRobot](https://huggingface.co/docs/lerobot), [integrate_hardware](https://huggingface.co/docs/lerobot/integrate_hardware).
- LeRobotDataset v3: [lerobot-dataset-v3](https://huggingface.co/docs/lerobot/lerobot-dataset-v3).
- LUCY ROS contract: **INTERFACE.md** and **EXTERNAL_CLIENTS.md** in this package.

# Control Panel as Microphone and Speakers

When the robot has no physical mic/speakers connected, the **web control panel** can act as the audio I/O: the user enables a “Use my microphone and speakers” (or similar) option, and the panel publishes the browser mic to `/mic_audio` and subscribes to `/audio` for playback. The speech pipeline and any other client use the same topics; they do not need to know whether the source is the robot or the panel.

---

## 1. Contract (same as robot audio)

- **Microphone (input):** Publish to **`/mic_audio`**  
  Type: `audio_common_msgs/msg/AudioStamped`
- **Playback (output):** Subscribe to **`/audio`**  
  Type: `audio_common_msgs/msg/AudioStamped`

When the feature is **on**, the control panel is the only publisher of `/mic_audio` and the only subscriber that plays `/audio` (see “Launch mode” below).

---

## 2. Message format

**`audio_common_msgs/msg/AudioStamped`**

| Field | Type | Description |
|-------|------|-------------|
| `header` | `std_msgs/Header` | `frame_id`, `stamp` (optional) |
| `audio.info` | `audio_common_msgs/AudioInfo` | See below |
| `audio.audio_data` | `audio_common_msgs/AudioData` | One of the data arrays filled |

**`audio.info`:** `format`, `channels`, `rate`, `chunk` (all int32).  
- `format`: PortAudio-style (e.g. 8 = paInt16, 1 = paFloat32). Commonly **8 (int16)**.  
- `channels`: 1 (mono) or 2 (stereo).  
- `rate`: Sample rate in Hz (e.g. 16000, 48000).  
- `chunk`: Number of samples per message (e.g. 1024).

**`audio.audio_data`:** Use **`int16_data`** or **`float32_data`**; leave others empty.  
- For **int16**: array length = `chunk * channels`.  
- For **float32**: same; values typically in [-1, 1].

Example (int16, mono, 16 kHz, 1024 samples):

```json
{
  "header": { "frame_id": "control_panel_mic", "stamp": { "sec": 0, "nanosec": 0 } },
  "audio": {
    "info": { "format": 8, "channels": 1, "rate": 16000, "chunk": 1024 },
    "audio_data": {
      "float32_data": [],
      "int32_data": [],
      "int16_data": [ 0, 0, ... ],
      "int8_data": [],
      "uint8_data": []
    }
  }
}
```

---

## 3. Transport: rosbridge

The control panel already connects to LUCY via **rosbridge** (WebSocket). Use that connection to:

1. **Publish** `AudioStamped` to `/mic_audio` at a steady rate (e.g. ~46–50 Hz for 1024 samples at 16 kHz ≈ 64 ms per chunk).
2. **Subscribe** to `/audio` and, in the callback, decode the message and play it with the Web Audio API (or equivalent).

rosbridge serializes ROS messages to JSON, so the panel sends/receives the structure above. Ensure the topic type is advertised as `audio_common_msgs/msg/AudioStamped` (rosbridge will use it for (de)serialization if needed).

---

## 4. Control panel implementation outline

1. **Toggle**  
   Add a user option (e.g. “Use my microphone and speakers”) that enables this mode.

2. **When enabled**  
   - **Mic:**  
     - Request `getUserMedia()` (audio).  
     - Create a script processor or audio worklet that produces chunks (e.g. 1024 samples) at the chosen rate.  
     - Convert to int16 (or float32) and fill `audio.audio_data.int16_data` (or `float32_data`).  
     - Set `audio.info`: `format` 8 for int16 or 1 for float32, `channels`, `rate`, `chunk`.  
     - Publish to `/mic_audio` at the appropriate interval (e.g. chunk_size / rate seconds).
   - **Playback:**  
     - Subscribe to `/audio` over rosbridge.  
     - In the callback, read `msg.audio.audio_data.int16_data` (or `float32_data`), `msg.audio.info.rate`, `msg.audio.info.channels`.  
     - Decode and queue to an `AudioContext` / `AudioBuffer` and play (e.g. with a short queue to smooth jitter).

3. **When disabled**  
   Stop publishing to `/mic_audio`, unsubscribe from `/audio`, release the mic stream.

4. **Rates**  
   The speech pipeline often expects 16 kHz mono for ASR. You can either publish at 16 kHz mono from the panel, or publish at a higher rate (e.g. 48 kHz stereo) and let the pipeline resample (if it does). Matching the pipeline’s expected format (see its config) avoids extra latency.

---

## 5. Launch mode (robot vs panel)

- **Robot mic/speakers:**  
  `ros2 launch lucy_bringup lucy.launch.py`  
  Then `audio_capturer` publishes `/mic_audio` and `audio_player` subscribes to `/audio`.

- **Control panel mic/speakers (no robot audio):**  
  (obsolete: audio is always on)  
  Do **not** start the robot audio nodes. Only the control panel publishes `/mic_audio` and subscribes to `/audio`. The speech pipeline still runs with `--lucy` and sees the same topics.

- Do **not** run both robot audio and panel audio at the same time for the same role (e.g. two publishers to `/mic_audio`), unless you explicitly design a mixer or single “active” source.

### Hearing playback in the browser (and your own voice)

To hear **your voice** played back in the browser, only the panel must publish to `/mic_audio`. You can still have the robot’s **player** running so `/audio` is subscribed and playback works:

1. **Launch with playback only (no robot mic):**  
   `ros2 launch lucy_bringup lucy.launch.py`  
   This starts the **audio_player_node** (subscribes to `/audio`) but **not** the audio_capturer, so the only publisher to `/mic_audio` is the control panel. You hear your voice on playback and the browser receives `/audio` via rosbridge.

2. **Control panel:** Open the panel, connect to the robot. Turn **Mic** and **Speakers (headset)** **ON**.

3. **Pipeline:** Run with `play_back_after_vad: true` and `connection.mic_sample_rate: 16000` in pipeline config (same machine as lucy_bringup so it sees `/mic_audio` and `/audio`).

**Note:**

- Audio is always on. `/mic_audio` and subscribes to `/audio` (playback only in the browser). Use if you don’t need the robot’s physical speaker.
- Robot capturer and player both run; `/mic_audio` has two publishers (robot + panel), so playback will be mixed/noise. Use only when you want the robot’s mic and speakers on the device.

---

## 6. Summary

| Responsibility | Action |
|----------------|--------|
| Control panel (when “use my mic/speakers” is on) | Publish browser mic → `/mic_audio`; subscribe `/audio` → play in browser. |
| Speech pipeline | Unchanged: `run_pipeline.py --lucy` (reads `/mic_audio`, writes `/audio`). |
| Launch | `ros2 launch lucy_bringup lucy.launch.py` (player always; capturer off by default → panel mic only). Use `robot_mic_enable:=true` to enable Jetson mic. |

The interface (topic names and message type) is the same as in **INTERFACE.md**; only the source of `/mic_audio` and the sink for `/audio` change.

---

## 7. Integration test: control panel → ROS 2 → speech_ai_pipeline

End-to-end check: browser mic (control panel) → `/mic_audio` → speech pipeline → `/audio` → browser speakers (control panel).

### Prerequisites

- LUCY workspace built, `lucy_ros_api` and `lucy_bringup` installed.
- `speech_ai_pipeline` cloned and deps installed; `config/config.yaml` present (e.g. copy from `config.example.yaml`).
- In **speech_ai_pipeline** `config/config.yaml`, set `connection.mic_sample_rate: 16000` when using the control-panel mic (panel publishes at 16 kHz). If you use the robot’s audio_capturer (48 kHz), use `48000`.
- Control panel dev server or build (e.g. `yarn dev --host` in `web_control_panel`).

### Step 1: Start LUCY

```bash
cd ~/lucy_ws
source install/setup.bash
ros2 launch lucy_bringup lucy.launch.py
```

Leave this running. By default only the audio player runs (no robot mic); turn **Mic** and **Speakers** ON in the control panel to use the panel as mic and hear playback in the browser.

### Step 2: Start the speech pipeline

In a **second terminal**:

```bash
source ~/lucy_ws/install/setup.bash
cd ~/speech_ai_pipeline
python3 run_pipeline.py --lucy
```

Optional: `--lucy --no-asr` for a quick loop without ASR. The pipeline subscribes to `/mic_audio` and publishes to `/audio` (and uses `/say` if configured).

### Step 3: Open the control panel and connect

1. Open the control panel in the browser (e.g. `https://<jetson-ip>:5173` or your dev URL).
2. Enter the ROS Bridge URL if needed (e.g. `wss://<jetson-ip>:9090` or the proxy path your app uses) and click **CONNECT**.
3. When connected, enable **microphone** (round mic button): allow browser mic access. The panel will publish to `/mic_audio`.
4. Optionally enable **speakers** (round headset button) so you hear the pipeline’s reply on `/audio`.

### Step 4: Verify

- **Topics (third terminal):**
  ```bash
  source ~/lucy_ws/install/setup.bash
  ros2 topic list | grep -E 'mic_audio|audio'
  ros2 topic hz /mic_audio
  ```
  You should see `/mic_audio` and `/audio`; `/mic_audio` should show a rate when the panel mic is on.

- **Pipeline:** In the pipeline terminal you should see logs about every 5 s (`mic: receiving`), then `VAD: end of utterance`, `ASR: <text>`, and `reply sent: ...` when you speak and pause. If you see nothing, set `connection.mic_sample_rate: 16000` in `speech_ai_pipeline/config/config.yaml` (control panel sends 16 kHz).
- **Playback:** The pipeline sends only **text** to the `/say` action. To hear a spoken reply in the panel, a **TTS node** must run that handles `/say` and publishes to `/audio`; the panel then plays it. Without that, you see the reply text in the log but do not hear it in the browser.

### Troubleshooting

- **No `/mic_audio`:** Panel not connected or mic not enabled; check browser console and ROS connection.
- **Pipeline “no data”:** Check that the panel is publishing (e.g. `ros2 topic hz /mic_audio`) and that message type is `audio_common_msgs/msg/AudioStamped`.
- **No sound in browser:** Enable the **Speakers** button in the panel. Run the pipeline on the **same machine** as lucy_bringup. (Default is panel mic only; the robot’s player still runs for playback.)
- **Playback is noise, not my voice:** By default the robot capturer is off; only the panel publishes to `/mic_audio`. If you launched with `robot_mic_enable:=true`, relaunch without it so only the panel is the mic source.
- **Rosbridge URL:** If the panel is served behind a proxy that forwards to rosbridge, use the URL that reaches the WebSocket (e.g. same host as the panel with path `/rosbridge`).

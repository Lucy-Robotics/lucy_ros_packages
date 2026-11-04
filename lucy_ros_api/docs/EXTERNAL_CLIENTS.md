# Connecting External Clients to LUCY ROS

External processes (AI models, web apps, non-ROS controllers) can connect to the LUCY ROS system in two ways.

---

## Option 1: Direct ROS 2 connection

If the client runs on the same machine (or in the same ROS 2 DDS network):

- Use **rclpy** (Python) or **rclcpp** (C++) to subscribe/publish and call actions.
- Dependencies: ROS 2 Humble (or same distro as LUCY), `audio_common_msgs`.
- See **INTERFACE.md** for topic/action names and types.

Example (Python): create a node that subscribes to `/mic_audio` and publishes to `/audio` (e.g. a thin bridge used by the speech AI process).

**Control panel as mic/speakers:** When the robot has no physical audio devices, the web control panel can publish browser mic to `/mic_audio` and subscribe to `/audio` for playback (over rosbridge). See **CONTROL_PANEL_AUDIO.md** for the contract and implementation outline.

---

## Option 2: LUCY ROS API bridge (future)

A **bridge node** in this package can expose an HTTP or WebSocket API that:

- Accepts requests (e.g. “play this audio”, “say this text”, “get last image”).
- Translates them into ROS 2 publish/service/action calls.
- Optionally streams topic data (e.g. audio, images) back over WebSocket.

Then:

- Any non-ROS client talks to the bridge (e.g. `http://localhost:8080` or `ws://localhost:9090`) and does **not** need a ROS stack.
- The bridge is the only process that talks to ROS 2.

Bridge design (to be implemented): see package README and launch file when available.

# Testing (lucy_bringup / lucy_ros_packages)

This document describes how to run **tests that belong to this repository** (lucy_ros_packages) and how to **verify the audio and generic interface** for external clients. It does not cover testing of other repositories.

## 1. Unit tests: lucy_ros_api

The generic interface and ROS adapter live in the `lucy_ros_api` package. Run their tests from the LUCY workspace:

```bash
cd ~/lucy_ws
source install/setup.bash
colcon test --packages-select lucy_ros_api
```

Or run pytest from the package source directory:

```bash
cd ~/lucy_ws/src/lucy_ros_packages/lucy_ros_api
PYTHONPATH=. python3 -m pytest test/ -v
```

See `lucy_ros_api/README.md` for details.

## 2. Integration: Verifying the audio interface

When you launch LUCY with audio enabled, the capturer publishes microphone data on `/mic_audio` and the player subscribes to `/audio`. External clients (any process using the generic interface or raw ROS topics) can use these topics without depending on a specific application.

### 2.1 Launch LUCY (audio always enabled)

```bash
cd ~/lucy_ws
source install/setup.bash
ros2 launch lucy_bringup lucy.launch.py
```

Audio nodes (capturer and player) are always started.

### 2.2 Verify topics

In another terminal (with the workspace sourced):

```bash
ros2 topic list
```

You should see `/mic_audio` (published by the capturer) and `/audio` (subscribed by the player). Optional checks:

```bash
ros2 topic hz /mic_audio
ros2 topic echo /mic_audio --no-arr
```

### 2.3 Testing with the generic interface

Clients that use the **generic interface** (see `lucy_ros_api/docs/GENERIC_INTERFACE.md`) connect via `LucyROSAdapter`: observations from `/mic_audio`, actions to `/audio` and `/say`. Any such client (in this repo or elsewhere) can be run against a live LUCY stack by ensuring the workspace is sourced so `lucy_ros_api` is available and LUCY is running. How to run a specific client is documented in that client’s repository.

See `README.md` for launch arguments (e.g. audio sample rate, devices).

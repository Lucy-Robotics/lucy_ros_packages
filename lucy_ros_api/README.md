# lucy_ros_api

**LUCY ROS interface** – API specification and optional bridge for external clients.

## Purpose

- Define the **ROS 2 API** (topics, services, actions) that external systems use to interact with LUCY.
- Allow any AI model, controller, or application to connect to the LUCY ROS system via a stable, documented contract.
- Optionally provide an **external API bridge** (HTTP/WebSocket) so non-ROS clients can talk to LUCY without running ROS.

## Contents

- **docs/INTERFACE.md** – Authoritative specification of the LUCY ROS interface (audio, TTS, cameras, joints).
- **docs/EXTERNAL_CLIENTS.md** – How to connect from outside ROS (direct ROS vs bridge).
- **docs/GENERIC_INTERFACE.md** – Generic controller/AI interface (LeRobot-style), speech subset, and ROS adapter.
- **Python package `lucy_ros_api`** – Generic interface implementation: `RobotInterface`, `LucyROSAdapter` (ROS), `MockSpeechAdapter` (testing). Use: `from lucy_ros_api.generic_interface import RobotInterface, LucyROSAdapter, MockSpeechAdapter`.
- Optional: bridge node and launch file (when implemented).

## Build / install

Part of `lucy_ros_packages`. Build and source the workspace so the Python package is on PYTHONPATH.

```bash
cd ~/lucy_ws
colcon build --packages-select lucy_ros_api
source install/setup.zsh
```

Documentation is installed under `share/lucy_ros_api/docs/`. The generic interface is installed as a Python package (import `lucy_ros_api.generic_interface`). Dependencies: `rclpy`, `std_msgs`, `audio_common_msgs` (and `python3-numpy` for the adapters).

## Tests

Unit tests cover the generic interface (base contract, `MockSpeechAdapter`). No ROS or LUCY required.

**From the workspace (with build):**

```bash
cd ~/lucy_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select lucy_ros_api --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select lucy_ros_api
```

**From the package source (pytest only):**

```bash
cd ~/lucy_ws/src/lucy_ros_packages/lucy_ros_api
PYTHONPATH=. python3 -m pytest test/ -v
```

Tests import `lucy_ros_api.generic_interface` (package lives in `lucy_ros_api/` in the project root).

## License

GPL-3.0. Copyright Sentience Robotics Team.

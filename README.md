# lucy_ros_packages

ROS 2 **Jazzy** repository for **Lucy** (Sentience Robotics): runtime bringup, `ros2_control` hardware integration, and camera tooling. This is a **multi-package** repo; each subdirectory under `src/` in your colcon workspace is one `ament` package.

## What lives here

| Package | One-line role |
|---------|----------------|
| [**lucy_bringup**](lucy_bringup/) | Jetson **system launch**: **`lucy_modbus_bridge`**, **`web_ros_api`** (rosbridge + **`lucy_config_pipeline`**), RealSense, `camera_ros`, [`lucy_ros2_control`](lucy_ros2_control/) bringup; development launches compose the web stack with the robot URDF package. |
| [**lucy_ros2_control**](lucy_ros2_control/) | **Hardware** `ros2_control` plugin (`LucySystemHardware` → SHM registers), shared clamp/mapping helpers. |
| [**lucy_modbus_bridge**](lucy_modbus_bridge/) | Relays SHM dirty registers to Modbus RTU over USB CDC. |
| [**lucy_config_generator**](lucy_config_generator/) | **Config pipeline**: reads robot hardware YAML and emits Rust firmware YAML, `ros2_control` xacro, and `controllers.yaml`. |
| [**lucy_config_pipeline**](lucy_config_pipeline/) | **Config store + `ConfigurePipeline` action**: validate, generate, Cargo build/flash RP2040 firmware via `picotool` [README](lucy_config_pipeline/README.md). |
| [**camera_ros**](camera_ros/) | GStreamer-based **MJPEG** → `sensor_msgs/CompressedImage`; client-aware activation. |

Package names match directories (`<name>` in each `package.xml`).

## How this repo fits the platform

- **Robot model, RViz, and Gazebo** live in the sibling repo **[thais_urdf](https://github.com/Sentience-Robotics/thais_urdf)** (package name `thais_urdf`). **`lucy_bringup`** owns **rosbridge + hardware config** (**`web_ros_api.launch.py`**) and **`lucy.launch.py`** (composition via **`real`**, **`rviz`**, **`gazebo`**). `lucy_ros2_control` expects URDF/xacro and meshes from **`thais_urdf`** when using default paths.
- **Web control panel** and teleop semantics are **not** in this repo; they consume the same topics/controllers documented in lucy_ws docs.

## Requirements

- **OS**: Ubuntu 24.04
- **ROS**: [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html).
- **Per-package extras**: Jetson-typical USB video and audio stacks for bringup; RealSense SDK stack for `realsense2_camera`; USB serial for Modbus boards. See each package README and `lucy_bringup/REALSENSE.md`.

## Picotool and passwordless sudo

The **`lucy_config_pipeline`** flash phase runs **`sudo picotool`**. Copy-paste **sudoers** setup (Ubuntu 24.04) lives in **[lucy_config_pipeline/README.md — Passwordless sudo for picotool](lucy_config_pipeline/README.md#passwordless-sudo-for-picotool)** (same repository; no `../` path).

## Building (colcon workspace)

Treat this repository as **`src/lucy_ros_packages`** (clone the contents into that folder) **or** clone in place so that packages are direct children of your workspace `src/`:

```text
lucy_ws/
└── src/
    ├── lucy_ros_packages/    # this repo: lucy_bringup, lucy_ros2_control, lucy_config_generator, lucy_config_pipeline, camera_ros
    └── thais_urdf/           # robot description + sim launches (separate repo)
```

Example build:

```bash
source /opt/ros/jazzy/setup.bash
cd lucy_ws
colcon build --symlink-install \
  --packages-select lucy_bringup lucy_ros2_control lucy_config_generator lucy_config_pipeline camera_ros
source install/setup.bash
```

To include simulation/description from the other repo:

```bash
colcon build --symlink-install \
  --packages-select lucy_bringup lucy_ros2_control lucy_config_generator lucy_config_pipeline camera_ros thais_urdf
```

## Quick start

| Goal | Entry point |
|------|-------------|
| Full Jetson stack (agents, rosbridge, cameras, `ros2_control`) | `ros2 launch lucy_bringup lucy.launch.py` |
| Control stack only (defaults assume `thais_urdf` layout in workspace) | `ros2 launch lucy_ros2_control control.launch.py` |
| USB MJPEG camera node | `ros2 launch camera_ros camera.launch.py` |

Tmux helpers ship with `lucy_bringup` (`system_scripts/`); see [lucy_bringup/README.md](lucy_bringup/README.md).

## Tests and coverage (local)

**Dependencies:** `python3-pytest-cov` (e.g. `sudo apt install python3-pytest-cov` on Ubuntu).

From your **workspace root** (e.g. `lucy_ws`), with packages under `src/lucy_ros_packages/`:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install \
  --packages-select camera_ros lucy_bringup lucy_ros2_control lucy_config_generator \
  --cmake-args -DBUILD_TESTING=ON
source install/setup.bash

colcon test --packages-select camera_ros lucy_bringup lucy_ros2_control lucy_config_generator --event-handlers console_direct+
colcon test-result --verbose
```

**Coverage** (same `source install/setup.bash` as above; open `build/coverage_html/.../index.html` after `--cov-report=html`):

```bash
mkdir -p build/coverage_reports build/coverage_html
python3 -m pytest src/lucy_ros_packages/camera_ros/test/ \
  --cov=src/lucy_ros_packages/camera_ros/scripts \
  --cov-report=term-missing \
  --cov-report=xml:build/coverage_reports/camera_ros.xml \
  --cov-report=html:build/coverage_html/camera_ros

python3 -m pytest src/lucy_ros_packages/lucy_bringup/test/ \
  --cov=src/lucy_ros_packages/lucy_bringup/launch \
  --cov-report=term-missing \
  --cov-report=xml:build/coverage_reports/lucy_bringup.xml \
  --cov-report=html:build/coverage_html/lucy_bringup

```

**Meaningful line coverage** is mostly from **`camera_ros/scripts`**; bringup tests only **byte-compile** launch files.

## CI

GitHub Actions (`.github/workflows/ci.yml`) runs **`rosdep`**, **`colcon build`**, **`colcon test`**, then **`pytest-cov`** over Python tests, producing Cobertura XML under `ws/build/coverage_reports/` for **`camera_ros`**, **`lucy_bringup`**, and **`lucy_config_generator`**. Reports are uploaded to [**Codecov**](https://codecov.io) when **`CODECOV_TOKEN`** is set on the repo, and HTML/XML are attached as workflow artifacts (`coverage-lucy_ros_packages`). See [`docs/DEVELOPER.md`](docs/DEVELOPER.md) §5.

## Documentation map

| Doc | Audience |
|-----|----------|
| This file | Anyone cloning **this** repository |
| [**docs/DEVELOPER.md**](docs/DEVELOPER.md) | **Contributors** — build, CI, package internals, extension checklist |
| [**docs/ROS2_CONTROL.md**](docs/ROS2_CONTROL.md) | **ros2_control** — SHM/Modbus path, `LucySystemHardware`, launches |
| [**docs/INTEGRATION_MODBUS.md**](docs/INTEGRATION_MODBUS.md) | **Integration checklist** — generate → Cargo build → flash → bridge |
| [lucy_bringup/README.md](lucy_bringup/README.md) | Operators and integrators (devices, tmux, launch args) |
| [lucy_ros2_control/README.md](lucy_ros2_control/README.md) | Control stack quick start |
| [**lucy_config_generator/README.md**](lucy_config_generator/README.md) | Hardware YAML → firmware YAML, `ros2_control` xacro, controllers |
| [**lucy_config_pipeline/README.md**](lucy_config_pipeline/README.md) | Config services + pipeline action (build/flash); [passwordless sudo for picotool](lucy_config_pipeline/README.md#passwordless-sudo-for-picotool) |
| [lucy_modbus_bridge/README.md](lucy_modbus_bridge/README.md) | SHM → Modbus RTU bridge |
| [camera_ros/README.md](camera_ros/README.md) | Camera topics, parameters, troubleshooting |

If these repos live under **`lucy_ws`**, see **`lucy_ws/docs/developer_lucy_packages.md`** (index into each repo’s `docs/DEVELOPER.md`) and **`lucy_ws/docs/simulation_and_visualization.md`** (full-stack pipeline).

## License

Packages in this repository are licensed under **GPL-3.0** unless a subdirectory states otherwise. See each package and the repository **`LICENSE`** file if present.

## Maintainer

Sentience Robotics Team — `contact@sentience-robotics.fr` (see `package.xml` files).

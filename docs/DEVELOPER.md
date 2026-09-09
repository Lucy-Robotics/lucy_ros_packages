# Developer guide — `lucy_ros_packages`

ROS 2 **Jazzy**. For **contributors** who change launch files, nodes, `ros2_control` config, or CI in **this repository only**.

**Sibling robot description / sim launches**: `../thais_urdf/docs/DEVELOPER.md` when both repos live under the same workspace `src/` (e.g. `lucy_ws/src`). On GitHub: [Sentience-Robotics/thais_urdf](https://github.com/Sentience-Robotics/thais_urdf) → `docs/DEVELOPER.md`.

Conventions follow common packaging practice ([REP-149](https://www.ros.org/reps/rep-0149.html): accurate `package.xml`, install rules, documented launch entry points).

---

## 1. Packages in this repo

| Package | Responsibility |
|---------|----------------|
| `lucy_bringup` | Jetson **system bringup**: `lucy_modbus_bridge`, `rosbridge_server`, `camera_ros`, RealSense, control include. |
| `lucy_ros2_control` | **Hardware** `ros2_control`: `LucySystemHardware` plugin (SHM register table). |
| `lucy_modbus_bridge` | SHM → Modbus RTU over USB serial (one node per board). |
| `lucy_config_generator` | Hardware YAML → RP2040 `config_*.yaml`, `ros2_control` xacro, `controllers.yaml`. |
| `lucy_config_pipeline` | Config store + `ConfigurePipeline` (validate → generate → Cargo build → flash → reload). |
| `camera_ros` | MJPEG → `sensor_msgs/msg/CompressedImage`; GStreamer pipeline; pytest. |

---

## 2. Layout

```text
lucy_ros_packages/
├── docs/
│   ├── DEVELOPER.md         # this file
│   └── ROS2_CONTROL.md      # ros2_control concepts + Lucy implementation
├── lucy_bringup/
├── lucy_ros2_control/
├── lucy_modbus_bridge/
├── lucy_config_generator/
├── lucy_config_pipeline/
└── camera_ros/
```

---

## 3. Build, test, install

```bash
cd lucy_ws   # or your colcon workspace root
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install \
  --packages-select lucy_bringup lucy_ros2_control lucy_config_generator camera_ros
source install/setup.bash
```

**Hygiene**

- **`package.xml`**: `exec_depend` for anything launched at runtime; `depend` / `buildtool_depend` for build-time Python/C++.
- **`CMakeLists.txt`**: install everything consumers need under `share/${PROJECT_NAME}/` (or `lib/`) so `ros2 pkg prefix <pkg>` is sufficient.
- **Tests**: enable `BUILD_TESTING`; extend `ament_lint` / pytest when behavior changes.
- **Version**: bump `<version>` in `package.xml` on visible behavior or API changes.

---

## 4. Package reference

### `lucy_bringup`

| Item | Detail |
|------|--------|
| **Launch** | `ros2 launch lucy_bringup lucy.launch.py` |
| **Args** | `device0`, `device1` (default `/dev/ttyACM0`, `/dev/ttyACM1`); audio args declared but audio nodes are **commented out** in `lucy.launch.py`; RealSense via `realsense.launch.py`. |
| **Scripts** | `system_scripts/*.sh` → installed under `lib/lucy_bringup`. |
| **Runtime deps** | `lucy_modbus_bridge`, `lucy_ros2_control`, `rosbridge_server`, `camera_ros`, `audio_common`, `realsense2_camera`, `lucy_config_pipeline`, `launch`, `launch_ros`. |

**Developers**

- `lucy_ros2_control` is included after a **3 s** `TimerAction` so serial/rosbridge can settle; if you change ordering, re-validate hardware bringup.
- See `REALSENSE.md`, `AUDIO.md` in this package for subsystems.

### `lucy_ros2_control`

| Item | Detail |
|------|--------|
| **Launch** | `ros2 launch lucy_ros2_control control.launch.py` |
| **Plugin** | `lucy_ros2_control.xml` → `lucy_ros2_control/LucySystemHardware`; C++ implementation under `hardware/`. |
| **Config** | `config/lucy_controllers.yaml` — must match joint names in **thais_urdf** xacro (`inmoov_ros2_control.xacro`) when you maintain controllers here; generated **`thais_urdf`** `controllers.yaml` must stay aligned when using **`lucy_config_generator`**. |
| **Architecture** | **[docs/ROS2_CONTROL.md](ROS2_CONTROL.md)** — general ros2_control → Lucy topics, YAML, launches, pitfalls. |

**When you change controllers or joints**

1. Edit `config/lucy_controllers.yaml`.
2. Update **`thais_urdf`** ros2_control xacro in the **same change set** (or coordinated PRs).
3. Align any external UI / teleop joint lists (e.g. control panel config).

### `camera_ros`

| Item | Detail |
|------|--------|
| **Launch** | `ros2 launch camera_ros camera.launch.py` (`fps`, `device`, USB ids, …). |
| **Tests** | `colcon test --packages-select camera_ros` with `BUILD_TESTING=ON`. |

### `lucy_config_generator`

| Item | Detail |
|------|--------|
| **CLI** | `ros2 run lucy_config_generator generate …` (see package **README**). |
| **Tests** | `colcon test --packages-select lucy_config_generator` — golden outputs for firmware YAML, xacro, controllers. |

---

## 5. CI

`.github/workflows/ci.yml` runs in `osrf/ros:jazzy-desktop` on **pull_request** and on **push** to **`main` / `master` / `dev`** only (avoids duplicate runs when a PR branch is pushed):

- **`rosdep install --from-paths src`** for `camera_ros`, `lucy_bringup`, `lucy_ros2_control`, `lucy_config_generator`
- **`colcon build`** with `BUILD_TESTING=ON`
- **`colcon test`** — ament linters, `camera_ros` pytests, **`lucy_bringup`** launch `py_compile` tests, **`lucy_ros2_control`** YAML tests, **`lucy_config_generator`** golden tests
- **`pytest-cov`** — `camera_ros` (`scripts/`), `lucy_bringup` (`launch/`), `lucy_ros2_control` (`test/`), `lucy_config_generator` (`lucy_config_generator/`) → Cobertura XML + HTML under `ws/build/coverage_reports/`; **Codecov** flag `lucy_ros_packages` (optional **`CODECOV_TOKEN`**)

Local commands: **README.md** → *Tests and coverage (local)*.

---

## 6. Extension checklist (`lucy_ros_packages`)

1. New **runtime** dependency → correct `package.xml` in the affected package.
2. New **launch or config** → `install()` in that package’s `CMakeLists.txt`.
3. New **topic/service** → document in the package `README.md`.
4. **Control / joints** → always coordinate with **`thais_urdf`** (see its `docs/DEVELOPER.md`).

---

## 7. Quick commands

| Goal | Command |
|------|---------|
| Full Jetson stack | `ros2 launch lucy_bringup lucy.launch.py` |
| Control stack only | `ros2 launch lucy_ros2_control control.launch.py` (requires **`thais_urdf`** installed in overlay — provides default URDF share) |
| USB camera | `ros2 launch camera_ros camera.launch.py` |
| RViz / Gazebo + web panel | **`lucy_bringup`** **`lucy.launch.py`** with **`rviz`**, **`gazebo`**, **`real`** (see **`lucy_ws/README.md`**) |
| URDF + RViz/Gazebo without web | **`thais_urdf`** **`control.launch.py`** + **`rviz_standalone.launch.py`**, or **`gazebo.launch.py`** |
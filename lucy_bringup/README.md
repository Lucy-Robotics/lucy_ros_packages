# Lucy Bringup Package

System launch files and scripts for the Lucy robot on NVIDIA Jetson AGX Orin.

## Overview

**`lucy.launch.py`** is the **single main entry**: it **always** includes **`web_ros_api.launch.py`** (rosbridge + **lucy_config_pipeline** for the control panel). Optional pieces are controlled by launch arguments (defaults match “full Jetson stack without RViz or Gazebo”):

| Argument | Default | When true / meaning |
|----------|---------|---------------------|
| **`real`** | `true` | `lucy_modbus_bridge` per board with `serial_id`, USB webcam (**`camera_ros`**), RealSense |
| **`rviz`** | `false` | RViz2 with **`robot_package`** RViz config (`use_sim_time:=false`). If **`gazebo:=true`**, forwarded as **`start_rviz`** to **`thais_urdf/gazebo.launch.py`** (no duplicate RViz). |
| **`gazebo`** | `false` | Include **`thais_urdf/gazebo.launch.py`**. **Requires `real:=false`** or launch aborts with **`RuntimeError`**. |
| **`robot_package`** | `thais_urdf` | **`control.launch.py`**, config paths, RViz config share |
| **`config_dir`** | *(empty)* | Override hardware YAML dir for **lucy_config_pipeline** |
| **`urdf_path`**, **`base_path`** | *(see launch file)* | Forwarded to **`thais_urdf`** Gazebo when **`gazebo:=true`** |

Audio nodes are not wired in **`lucy.launch.py`** today (reserved for future use).

## Building

From your colcon workspace (e.g. `src/lucy_ros_packages/lucy_bringup`):

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select lucy_bringup
source install/setup.bash
```

### Using the tmux Launcher (Recommended for Development)

Scripts resolve the colcon workspace as: **`LUCY_WS`** (if set and the directory exists), else from the script install path (`install/lib/lucy_bringup`) or source layout (`…/lucy_bringup/system_scripts`), else **`~/lucy_ws`**. The Vite app is started from **`${WORKSPACE}/src/lucy_control_panel`**.

```bash
# Launch everything
~/launch_lucy.sh

# Check system status (includes config pipeline + control panel probes)
~/check_lucy.sh

# Stop everything
~/stop_lucy.sh
```

### Using ROS2 Launch Directly

```bash
# Source workspace
source ~/lucy_ws/install/setup.zsh

# Default Jetson + panel (no RViz / no Gazebo)
ros2 launch lucy_bringup lucy.launch.py

# Jetson + RViz + panel
ros2 launch lucy_bringup lucy.launch.py rviz:=true

# Dev + panel + control + RViz (no Modbus bridges / cameras)
ros2 launch lucy_bringup lucy.launch.py real:=false rviz:=true

# Gazebo sim + panel (``rviz:=false`` = headless Gazebo)
ros2 launch lucy_bringup lucy.launch.py gazebo:=true real:=false

# Custom serial devices or robot package
ros2 launch lucy_bringup lucy.launch.py device0:=/dev/ttyACM2 device1:=/dev/ttyACM3
ros2 launch lucy_bringup lucy.launch.py robot_package:=my_robot_urdf
```

### Stopping (`stop_lucy.sh`)

The script interrupts the ROS pane (twice), stops the web pane, kills the tmux session, then runs a **fallback cleanup** for common orphaned processes (`rosbridge_websocket*`, `lucy_modbus_bridge` / `modbus_bridge_node`). It does **not** guarantee **every** node on the machine is gone: nodes started in another shell, another host on the same `ROS_DOMAIN_ID`, or names that linger briefly in discovery can still appear in `ros2 node list`. Use `ros2 node list` after stopping; if needed, stop other terminals or match remaining processes with `pgrep -af ros2`.

## Architecture

### tmux Layout

```
┌─────────────────────────────────────────────────────────────┐
│ Pane 0: ROS2 Nodes (Top 60%)                                │
│  ├─ lucy_modbus_bridge (per board)                          │
│  ├─ rosbridge_server                                        │
│  ├─ audio_capturer_node                                     │
│  └─ audio_player_node                                       │
│  └─ realsense2_camera                                       │
├──────┬──────────────────────────────────────────────────────┤
│ Web  │ Pane 2: Debug Terminal                               │
│ ~12% │ ~88%                                                 │
└──────┴──────────────────────────────────────────────────────┘
```

### tmux Controls

**Navigation:**
- **Detach (keep running)**: `Ctrl+B` then `D`
- **Switch panes**: `Ctrl+B` then arrow keys
- **Cycle panes**: `Ctrl+B` then `O`
- **Zoom pane**: `Ctrl+B` then `Z` (toggle fullscreen)
- **Reattach**: `tmux attach -t lucy`

**Scrolling (Copy Mode):**
- **Enter scroll mode**: `Ctrl+B` then `[`
- **Navigate**: Arrow keys, `PgUp`/`PgDn`, mouse wheel (if enabled)
- **Search forward**: `/` then type search term
- **Search backward**: `?` then type search term
- **Exit scroll mode**: `q` or `Esc`

## Launch Arguments

**`lucy.launch.py`:** `device0`, `device1`, `robot_package`, `config_dir`, **`real`**, **`rviz`**, **`gazebo`**, **`urdf_path`**, **`base_path`** (see **Overview** table). **`gazebo:=true`** with **`real:=true`** aborts at parse time.

**`realsense.launch.py`** is included as-is when **`real:=true`**; tune that file or wrap it if you need serial overrides.

## System Requirements

- Ubuntu 24.04 on NVIDIA Jetson AGX Orin
- ROS 2 Jazzy
- tmux
- Two RP2040 controllers connected via USB
- Intel RealSense D435i camera (see [REALSENSE.md](REALSENSE.md) for details)

## Troubleshooting

### Serial Devices Not Found

```bash
# List USB devices
ls -l /dev/ttyACM*

# Check which device is which
ros2 topic echo /trace_publisher
```

### Web Interface Not Starting

Check if the web directory exists:
```bash
ls ~/web_control_panel
```

### ROS Nodes Not Communicating

```bash
# Check all nodes
ros2 node list

# Check topics
ros2 topic list

# Echo joint commands
ros2 topic echo /actuators/right_arm
ros2 topic echo /actuators/left_arm

# Check audio topics
ros2 topic echo /audio
ros2 topic hz /audio
```

### Audio Underrun Warnings

**Note:** PortAudio underrun warnings are **normal and expected** when:
- No audio is being published to `/audio` topic
- Audio source stops temporarily
- System is idle

These are informational warnings, not errors. The system continues to function normally. You can safely ignore them. They will stop when audio data starts flowing again.

## Files

```
lucy_bringup/
├── launch/
│   ├── lucy.launch.py                    # Main entry (``real``, ``rviz``, ``gazebo``, …)
│   ├── web_ros_api.launch.py             # rosbridge + lucy_config_pipeline only
│   └── realsense.launch.py               # RealSense D435i camera launch file
├── system_scripts/
│   ├── launch_lucy.sh          # tmux launcher
│   ├── stop_lucy.sh            # Graceful shutdown
│   ├── check_lucy.sh           # Health check
│   └── lucy_workspace.zsh.inc  # Shared workspace path resolution (sourced by scripts)
├── CMakeLists.txt
├── package.xml
├── README.md                    # This file
└── REALSENSE.md                # RealSense D435i integration documentation
```

## Symlinks

Point `~/` scripts at the **installed** copies so they sit next to `lucy_workspace.zsh.inc` (required for path resolution):

```bash
ln -sf ~/lucy_ws/install/lib/lucy_bringup/launch_lucy.sh ~/launch_lucy.sh
ln -sf ~/lucy_ws/install/lib/lucy_bringup/stop_lucy.sh ~/stop_lucy.sh
ln -sf ~/lucy_ws/install/lib/lucy_bringup/check_lucy.sh ~/check_lucy.sh
```

You can also run them directly from `lucy_ws/src/lucy_ros_packages/lucy_bringup/system_scripts/` after `colcon build`.

## Camera System

The Lucy robot uses an **Intel RealSense D435i** stereo depth camera for vision and depth sensing.

**Key Features:**
- Color stream: 1920x1080 @ 30fps
- Depth stream: 1280x720 @ 30fps
- Aligned depth-to-color images
- IMU data (accelerometer + gyroscope @ 400Hz)
- Spatial and temporal filters for depth quality

**Topics:** All camera topics are published under `/realsense/` namespace.

For complete documentation, installation instructions, troubleshooting, and usage examples, see **[REALSENSE.md](REALSENSE.md)**.

## License

GPL-3.0 - See LICENSE file for details.

Copyright 2025 Sentience Robotics Team


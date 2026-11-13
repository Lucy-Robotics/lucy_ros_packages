# camera_ros — USB MJPEG camera publisher

Zero-copy **MJPEG** → `sensor_msgs/CompressedImage` for **ROS 2 Jazzy**, aimed at **NVIDIA Jetson** (AGX Orin) setups. Uses a **GStreamer** `v4l2src` pipeline and OpenCV with `CAP_GSTREAMER` (no software JPEG re-encode on the hot path when the camera outputs MJPEG).

The stack targets a **USB UVC webcam** exposed as `/dev/video*`. If you do not pass explicit USB IDs, the node tries to pick a device whose `v4l2-ctl --list-devices` label matches **`webcamproduct`** / **`usb-webcam`** (see `find_camera_by_id()` in `scripts/camera_publisher.py`); otherwise it uses the `device` parameter (default `/dev/video0`).

## Features

- MJPEG capture via GStreamer (`v4l2src` → `jpegparse` → `appsink`)
- Optional automatic start/stop from global **`/client_count`** (`std_msgs/Int32`) when `count > 0` / `== 0`
- Companion **`camera_stream_controller`** node (same launch file) that toggles streaming via services
- Launch parameters: `fps`, `device`, `vendor_id`, `product_id`, `serial_number`

## Requirements

- Ubuntu 24.04 / Jetson (typical) or similar with V4L2
- ROS 2 Jazzy
- Python 3.10+
- OpenCV built with **GStreamer** support (`cv2.CAP_GSTREAMER`)
- GStreamer 1.x, `v4l-utils` (`v4l2-ctl`) for enumeration and tuning

## Building

From your colcon workspace (e.g. `src/lucy_ros_packages/camera_ros`):

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select camera_ros
source install/setup.bash
```

## Quick start

**Prerequisites:** source `/opt/ros/jazzy/setup.*` then `install/setup.*`.

Starts **`camera_publisher`** and **`camera_stream_controller`**:

```bash
ros2 launch camera_ros camera.launch.py
```

**With parameters:**

```bash
ros2 launch camera_ros camera.launch.py fps:=20.0 device:=/dev/video0
ros2 launch camera_ros camera.launch.py vendor_id:=0x046d product_id:=0x0825
```

### Launch arguments

| Argument | Default | Role |
|----------|---------|------|
| `fps` | `10.0` | Max publish rate (clamped 1–30 in code when changed via `set_fps`) |
| `device` | `/dev/video0` | V4L2 device if auto-detection does not override |
| `vendor_id` | `""` | Optional USB vendor match (sysfs), e.g. `0x046d` |
| `product_id` | `""` | Optional USB product match |
| `serial_number` | `""` | Reserved / optional serial match path in code |

### Topics

| Name | Type | Direction |
|------|------|-----------|
| `/ext_camera/jpg` | `sensor_msgs/msg/CompressedImage` | **Published** (`format`: `jpeg`) when streaming |
| `/client_count` | `std_msgs/msg/Int32` | **Subscribed** by both nodes to gate streaming |

Publishing is active only while **`is_streaming`** is true (driven by `/client_count` and/or services).

### Services (on node **`camera_publisher`**)

| Service | Type |
|---------|------|
| `/camera_publisher/start_streaming` | `std_srvs/srv/SetBool` |
| `/camera_publisher/stop_streaming` | `std_srvs/srv/SetBool` |
| `/camera_publisher/get_client_count` | `camera_ros/srv/GetInt` — returns current cached client count |

Stop streaming before killing the node if you want a clean shutdown path:

```bash
ros2 service call /camera_publisher/stop_streaming std_srvs/srv/SetBool "{data: true}"
```

### Check rate

```bash
ros2 topic hz /ext_camera/jpg
```

## Configuration (code)

There is no standalone `GST_PIPELINE` constant. The pipeline is built in **`CameraPublisher.init_cap()`** in `scripts/camera_publisher.py`:

- Default stream request: **1920×1080**, **30 fps**, **MJPEG** (`input/jpeg`,…), device from `self.camera_device`.
- Module-level defaults at the top of the same file: **`FPS`**, **`CAMERA_DEVICE`**.

To change resolution or framerate contract, edit the string passed to `cv2.VideoCapture(..., cv2.CAP_GSTREAMER)` inside **`init_cap()`**, then rebuild.

## Performance

1. **Jetson:** `sudo jetson_clocks` when you need locked high performance.
2. **List devices / formats:**

   ```bash
   v4l2-ctl --list-devices
   v4l2-ctl --device=/dev/video0 --list-formats-ext
   ```

3. **CPU / bandwidth:** lower `fps` launch arg or reduce width/height in `init_cap()` if the USB sensor overloads the bus.

## Troubleshooting

**No image / open failed**

```bash
ls -l /dev/video*
v4l2-ctl --list-devices
# If the wrong node is chosen, set device explicitly:
ros2 launch camera_ros camera.launch.py device:=/dev/videoN
```

**Driver / uvc**

```bash
sudo modprobe uvcvideo
```

**High CPU**

- Lower `fps`.
- Use a lighter resolution in `init_cap()`.
- Close other captures on the same device.

**Node exits while streaming**

Prefer `Ctrl+C` in the terminal running launch, or call **`/camera_publisher/stop_streaming`** first.

## Architecture

```text
USB V4L2 device (e.g. /dev/videoN)
    → GStreamer: v4l2src ! image/jpeg,... ! jpegparse ! appsink
    → OpenCV VideoCapture (CAP_GSTREAMER)
    → CompressedImage on /ext_camera/jpg
```

`/client_count` and/or **`camera_stream_controller`** ↔ **`start_streaming` / `stop_streaming`** control whether frames are published.

## Nodes

| Script | Node name | Role |
|--------|-----------|------|
| `camera_publisher.py` | `camera_publisher` | Capture + publish + services |
| `camera_stream_controller.py` | `camera_stream_controller` | Bridges `/client_count` to streaming services |

## API (Python)

**`CameraPublisher`** (`scripts/camera_publisher.py`):

- ROS parameters: `fps`, `device`, `vendor_id`, `product_id`, `serial_number`
- **`set_fps(fps)`** — clamps to \[1.0, 30.0\]

There is no `set_active()`; use services or `/client_count` / controller node.

## Contributing

1. Fork / branch  
2. Run tests: `colcon test --packages-select camera_ros`  
3. Validate on target hardware (UVC + MJPEG path)

## License

GPL-3.0 — see LICENSE in the repository.

## Maintainer

**Lucy Robotics** — [contact@lucy-robotics.com](mailto:contact@lucy-robotics.com)

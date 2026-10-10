# Intel RealSense D435i Integration

Complete documentation for the Intel RealSense D435i stereo depth camera integration in the Lucy ROS2 system.

> **Official ROS2 Wrapper**: This integration uses the [official realsense-ros](https://github.com/realsenseai/realsense-ros) ROS2 wrapper maintained by Intel RealSense. For complete documentation, see the [official repository](https://github.com/realsenseai/realsense-ros).

## Table of Contents

- [Overview](#overview)
- [Installation](#installation)
- [Camera Specifications](#camera-specifications)
- [Configuration](#configuration)
- [Usage](#usage)
- [Published Topics](#published-topics)
- [Integration with Lucy System](#integration-with-lucy-system)
- [Troubleshooting](#troubleshooting)
- [Performance Optimization](#performance-optimization)

## Overview

The Intel RealSense D435i is a stereo depth camera that provides:
- **RGB Color Stream**: High-resolution color images
- **Depth Stream**: Stereo depth information
- **IMU Data**: Accelerometer and gyroscope for motion tracking
- **Aligned Depth-to-Color**: Depth data aligned with color image pixels

## Installation

### Official Installation Guide for Jetson AGX Orin

**Tested System:**
- **Board**: Jetson AGX Orin
- **L4T Version**: R36.4.7 (L4T 36.4.7)
- **Kernel**: 5.15.148-tegra
- **Ubuntu**: 24.04 LTS
- **JetPack**: 6.0

**Important**: For Jetson AGX Orin with JetPack 6, the **libuvc-backend** installation method is required. This method avoids kernel patching and works on a wider range of platforms. See the [official libuvc-backend installation guide](https://github.com/realsenseai/librealsense/blob/master/doc/libuvc_installation.md).

### Step 1: Install Librealsense2 SDK (libuvc-backend Method)

**Prerequisites:**
- Make sure **no RealSense device is connected** before starting
- Internet connection required
- Network proxy settings configured if needed

**Installation steps:**

```bash
# Download the installation script
wget https://github.com/realsenseai/librealsense/raw/master/scripts/libuvc_installation.sh

# Make script executable
chmod +x ./libuvc_installation.sh

# Run the installation script
./libuvc_installation.sh
```

**Wait for completion:**
- The script will take some time to complete
- Wait until you see `Librealsense script completed` message

**After installation:**
1. Connect the RealSense device
2. Verify installation:
   ```bash
   rs-enumerate-devices
   ```

**Note**: This method uses the libuvc-backend which doesn't require kernel patching. It's the recommended approach for Jetson devices when kernel patching is not possible or fails. The installation script automatically handles udev rules setup.

### Step 2: Install ROS2 RealSense Packages

After installing the librealsense2 SDK, install the ROS2 wrapper packages. The official ROS2 wrapper is maintained at [realsense-ros](https://github.com/realsenseai/realsense-ros).

```bash
sudo apt-get update
sudo apt-get install -y \
    ros-jazzy-realsense2-camera \
    ros-jazzy-realsense2-camera-msgs \
    ros-jazzy-realsense2-description
```


**Official ROS2 Wrapper Repository**: [realsenseai/realsense-ros](https://github.com/realsenseai/realsense-ros)

**Important**: After installing both the librealsense2 SDK and ROS2 packages, you may encounter a "bad optional access" error if multiple librealsense installations exist (libuvc and ROS wrapper). The ROS wrapper version may be loaded by default, but it doesn't work with the libuvc-backend installation.

**Solution**: Add the libuvc installation path to `LD_LIBRARY_PATH` to prioritize it:

```bash
# Add to ~/.zshrc (or ~/.bashrc if using bash)
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH}
```

Then reload your shell configuration:
```bash
source ~/.zshrc  # or source ~/.bashrc
```

## Camera Specifications

For complete specifications, see the [official D435i product page](https://www.intelrealsense.com/depth-camera-d435i/).

**Key Specifications:**
- **Model**: D435i (with IMU)
- **USB ID**: `8086:0b3a`
- **Color**: 1920x1080 @ 30fps (maximum)
- **Depth**: 1280x720 @ 30fps (maximum)
- **IMU**: Accelerometer + Gyroscope @ 400Hz

## Configuration

### Current Configuration

The RealSense D435i is configured with optimal settings in `lucy_bringup/launch/realsense.launch.py`:

- **Color Stream**: 1920x1080 @ 30fps (maximum)
- **Depth Stream**: 1280x720 @ 30fps (maximum)
- **Aligned Depth**: Enabled (depth aligned to color image plane)
- **IMU**: Separate topics for accelerometer and gyroscope @ 400Hz
- **Point Cloud**: Disabled
- **Infrared**: Disabled
- **Filters**: Spatial + Temporal (recommended for depth quality)

For complete parameter documentation, see the [official realsense-ros parameters documentation](https://github.com/realsenseai/realsense-ros#parameters).

## Usage

### Launch RealSense Standalone

Launch only the RealSense camera:

```bash
# Source workspace
source ~/lucy_ws/install/setup.zsh

# Launch RealSense camera
ros2 launch lucy_bringup realsense.launch.py
```

### Launch with Custom Serial Number

If you have multiple RealSense cameras:

```bash
ros2 launch lucy_bringup realsense.launch.py serial_no:=<CAMERA_SERIAL_NUMBER>
```

### Launch Full Lucy System

The RealSense camera is automatically launched as part of the full Lucy system:

```bash
# Using launch script
~/launch_lucy.sh

# Or using ROS2 launch directly
ros2 launch lucy_bringup lucy.launch.py
```

## Published Topics

All topics are published under the `/realsense` namespace.

**For complete topic documentation**, see the [official realsense-ros repository](https://github.com/realsenseai/realsense-ros).

### Main Topics (This Configuration)

- `/realsense/color/image_raw` - RGB color images (1920x1080 @ 30fps)
- `/realsense/color/camera_info` - Color camera calibration
- `/realsense/depth/image_rect_raw` - Native depth images (1280x720 @ 30fps)
- `/realsense/depth/camera_info` - Depth camera calibration
- `/realsense/aligned_depth_to_color/image_raw` - Aligned depth (1920x1080 @ 30fps)
- `/realsense/aligned_depth_to_color/camera_info` - Aligned depth calibration
- `/realsense/gyro/sample` - Gyroscope data (400 Hz)
- `/realsense/accel/sample` - Accelerometer data (400 Hz)

### Useful Commands

```bash
# List all topics
ros2 topic list | grep realsense

# Check frame rates
ros2 topic hz /realsense/color/image_raw
ros2 topic hz /realsense/depth/image_rect_raw

# View images
ros2 run rqt_image_view rqt_image_view /realsense/color/image_raw
```

## Services and Actions

For complete documentation on available services and actions, see:
- [Available Services](https://github.com/realsenseai/realsense-ros#available-services)
- [Available Actions](https://github.com/realsenseai/realsense-ros#available-actions)

### Common Services

- `/realsense/realsense2_camera/hw_reset` - Reset the device
- `/realsense/realsense2_camera/device_info` - Get device information (serial number, firmware version, etc.)

**Example:**
```bash
ros2 service call /realsense/realsense2_camera/device_info realsense2_camera_msgs/srv/DeviceInfo
```

## Integration with Lucy System

### Launch File Integration

The RealSense camera is integrated into the main Lucy launch file (`lucy.launch.py`):

```python
# RealSense D435i Camera (replaces camera_ros)
realsense_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        PathJoinSubstitution([
            FindPackageShare('lucy_bringup'),
            'launch',
            'realsense.launch.py'
        ])
    ]),
    launch_arguments={
        'serial_no': LaunchConfiguration('realsense_serial'),
    }.items()
)
```

### Namespace

All RealSense topics use the `/realsense/...` namespace to avoid conflicts with other systems.

## Troubleshooting

For common issues and solutions, see the [official realsense-ros troubleshooting](https://github.com/realsenseai/realsense-ros) and [Intel RealSense support](https://support.intelrealsense.com/).

### Common Issues

**Camera Not Detected:**

**Other checks:**
- Ensure the installation script completed successfully
- Check USB connection: `lsusb | grep -i intel`
- Verify user is in `video` and `plugdev` groups
- Reload camera (unplug/replug) after installation

**IMU Data Not Publishing:**
- Verify camera permissions (the installation script should handle this)
- Check HID device permissions: `ls -la /dev/hidraw*`
- Ensure `enable_gyro` and `enable_accel` are set to `true` in launch file


### Alignment Issues

**Problem**: Aligned depth doesn't match color image.

**Solutions**:

1. **Verify alignment is enabled**:
   ```python
   'align_depth.enable': True,
   ```

2. **Check camera calibration**:
   - RealSense cameras are factory calibrated
   - If issues persist, may need recalibration

3. **Verify topics are synchronized**:
   ```bash
   # Check timestamps are close
   ros2 topic echo /realsense/color/image_raw --once | grep stamp
   ros2 topic echo /realsense/aligned_depth_to_color/image_raw --once | grep stamp
   ```

## Performance Optimization

### Jetson AGX Orin Optimizations

1. **Set CPU governor to performance**:
   ```bash
   sudo jetson_clocks
   ```

2. **Monitor system resources**:
   ```bash
   # CPU usage
   htop
   
   # Jetson stats
   tegrastats
   
   # ROS2 node CPU usage
   ros2 run resource_monitor resource_monitor
   ```

3. **USB 3.0 Connection**:
   - Ensure camera is connected to USB 3.0 port
   - Check: `lsusb -t` should show "5000M" speed

### Reducing CPU Usage

If CPU usage is too high:

1. **Lower resolution**:
   ```python
   'rgb_camera.profile': '1280x720x30',
   'depth_module.profile': '640x480x30',
   ```

2. **Lower frame rate**:
   ```python
   'rgb_camera.profile': '1920x1080x15',
   'depth_module.profile': '1280x720x15',
   ```

3. **Disable filters** (if depth quality is acceptable):
   ```python
   'filters': '',
   ```

### Memory Usage

RealSense node typically uses:
- ~200-300 MB RAM for image buffers
- Additional memory for filters and processing

Monitor with:
```bash
ros2 run resource_monitor resource_monitor
```

## Advanced Configuration

### Custom Filter Parameters

Edit `lucy_bringup/launch/realsense.launch.py` to customize filters:

```python
# Spatial filter - stronger smoothing
'spatial_filter.magnitude': 3,
'spatial_filter.smooth_alpha': 0.6,
'spatial_filter.smooth_delta': 30,

# Temporal filter - more aggressive temporal smoothing
'temporal_filter.alpha': 0.5,
'temporal_filter.delta': 30,
```

### Enable Point Cloud (if needed)

To enable point cloud generation:

```python
'enable_pointcloud': True,
```

This will publish:
- `/realsense/depth/color/points` (sensor_msgs/PointCloud2)

**Note**: Point cloud generation is CPU-intensive and disabled by default.

### Enable Infrared Streams (if needed)

To enable infrared streams:

```python
'enable_infra1': True,
'enable_infra2': True,
```

This will publish:
- `/realsense/infra1/image_rect_raw`
- `/realsense/infra2/image_rect_raw`

**Note**: Infrared streams are typically not needed when depth is enabled.

## References

### Official ROS2 Wrapper

- **[realsense-ros GitHub Repository](https://github.com/realsenseai/realsense-ros)** - Official ROS/ROS2 wrapper for RealSense cameras

### Official SDK and Installation

- **[libuvc-backend Installation Guide](https://github.com/realsenseai/librealsense/blob/master/doc/libuvc_installation.md)** - Official libuvc-backend installation for Jetson (recommended for JetPack 6)
- [librealsense GitHub Repository](https://github.com/realsenseai/librealsense) - Official SDK source code
- [libuvc Installation Script](https://github.com/realsenseai/librealsense/raw/master/scripts/libuvc_installation.sh) - Direct link to installation script


### Additional Resources

- [RealSense ROS2 Services and Actions](https://github.com/realsenseai/realsense-ros#available-services) - Complete list of available services and actions
- [RealSense ROS2 Parameters](https://github.com/realsenseai/realsense-ros#parameters) - Configuration parameters documentation

## Support

For issues and questions:
- Check the troubleshooting section above
- Review Intel RealSense documentation
- Check ROS2 RealSense wrapper issues on GitHub
- Contact: contact@lucy-robotics.com

---

**Last Updated**: December 2025  
**Camera Model**: Intel RealSense D435i  
**ROS2 Distribution**: Jazzy  
**Platform**: NVIDIA Jetson AGX Orin


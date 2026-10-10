# Audio System Documentation - Lucy Robot

**Version:** 1.0.0  
**Last Updated:** December 2024  
**Platform:** NVIDIA Jetson AGX Orin, Ubuntu 24.04 LTS, ROS 2 Jazzy

---

## Table of Contents

1. [Overview](#overview)
2. [Architecture](#architecture)
3. [Hardware Requirements](#hardware-requirements)
4. [Software Stack](#software-stack)
5. [Installation & Setup](#installation--setup)
6. [Configuration](#configuration)
7. [Usage](#usage)
8. [ROS2 Topics & Messages](#ros2-topics--messages)
9. [Node Details](#node-details)
10. [Troubleshooting](#troubleshooting)
11. [Performance Optimization](#performance-optimization)
12. [Best Practices](#best-practices)
13. [API Reference](#api-reference)

---

## Overview

The Lucy robot audio system provides **stereo audio capture and playback** capabilities for humanoid robotics applications. It is designed to work with the NVIDIA Jetson AGX Orin development kit's audio panel, supporting real-time audio processing for:

- **Voice interaction** - Capturing human speech for processing
- **Sound localization** - Using stereo microphones for spatial audio awareness
- **Audio feedback** - Playing sounds, speech synthesis, or music through stereo speakers
- **Audio streaming** - Real-time audio transmission over ROS2 topics

### Key Features

- ✅ **Stereo capture** - Dual microphone input (2 channels)
- ✅ **Stereo playback** - Dual speaker output (2 channels)
- ✅ **Low latency** - PortAudio-based processing optimized for embedded systems
- ✅ **ROS2 native** - Built on `audio_common` package (ROS 2 Jazzy compatible)
- ✅ **Configurable** - Sample rates, device selection, buffer sizes
- ✅ **Resilient** - Auto-respawn on node failures
- ✅ **Real-time** - Optimized for Jetson AGX Orin hardware

---

## Architecture

### System Diagram

```
┌────────────────────────────────────────────────────┐
│                  Jetson AGX Orin                   │
│                                                    |
│  ┌──────────────────────────────────────────────┐  │
│  │  Audio Hardware (Audio Panel)                │  │
│  │  ├─ Stereo Microphones (Input)               │  │
│  │  └─ Stereo Speakers (Output)                 │  │
│  └──────────────────────────────────────────────┘  │
│                          │                         │
│                          ▼                         │
│  ┌──────────────────────────────────────────────┐  │
│  │  ALSA / PortAudio Layer                      │  │
│  └──────────────────────────────────────────────┘  │
│                          │                         │
│                          ▼                         │
│  ┌──────────────────────────────────────────────┐  │
│  │  ROS2 Audio Nodes                            │  │
│  │  ├─ audio_capturer_node                      │  │
│  │  │   └─ Publishes: /audio (AudioStamped)     │  │
│  │  └─ audio_player_node                        │  │
│  │      └─ Subscribes: /audio (AudioStamped)    │  │
│  └──────────────────────────────────────────────┘  │
│                          │                         │
│                          ▼                         │
│  ┌──────────────────────────────────────────────┐  │
│  │  ROS2 Topics                                 │  │
│  │  └─ /audio (audio_common_msgs/AudioStamped)  │  │
│  └──────────────────────────────────────────────┘  │
└────────────────────────────────────────────────────┘
```

### Data Flow

1. **Capture Path:**
   ```
   Microphones → ALSA → PortAudio → audio_capturer_node → /audio topic
   ```

2. **Playback Path:**
   ```
   /audio topic → audio_player_node → PortAudio → ALSA → Speakers
   ```

### Component Overview

| Component | Purpose | Package |
|-----------|---------|---------|
| `audio_capturer_node` | Captures audio from microphones and publishes to ROS2 | `audio_common` |
| `audio_player_node` | Subscribes to audio topic and plays through speakers | `audio_common` |
| `audio_common_msgs` | ROS2 message definitions for audio data | `audio_common_msgs` |
| PortAudio | Cross-platform audio I/O library | System dependency |
| ALSA | Linux audio subsystem | System dependency |

---

## Hardware Requirements

### Required Hardware

1. **NVIDIA Jetson AGX Orin Development Kit**
   - Integrated audio panel
   - Stereo line-in (for microphones)
   - Stereo line-out (for speakers)

2. **Stereo Microphones**
   - 2-channel input device
   - Compatible with 3.5mm audio jack or USB or analog interface
   - Recommended: Condenser microphones with phantom power (if using USB interface)

3. **Stereo Speakers**
   - 2-channel output device
   - Compatible with 3.5mm audio jack or USB or analog audio interface
   - Recommended: Powered speakers or headphones

### Audio Panel Connections

- Jetson AGX Orin [Hardware layout](https://developer.nvidia.com/embedded/learn/jetson-agx-orin-devkit-user-guide/developer_kit_layout.html) for audio panel position
- Jetson AGX Orin [Carrier Board datasheet](https://developer.nvidia.com/assets/embedded/secure/jetson/agx_orin/jetson_agx_orin_devkit_carrier_board_specification_sp) for audio panel header pins specs

### USB Audio Interfaces (Optional)

For higher quality audio or additional channels, USB audio interfaces are supported:
- Any USB Audio Class compliant device

---

## Software Stack

### ROS2 Packages

| Package | Version | Purpose |
|---------|---------|---------|
| `audio_common` | Latest (mgonzs13) | Audio capture and playback nodes |
| `audio_common_msgs` | Latest | Audio message definitions |
| `lucy_bringup` | 1.0.0 | Launch files and system integration |

---

## Installation & Setup

### 1. Install System Dependencies

```bash
sudo apt update
sudo apt install -y \
    libportaudio2 \
    libportaudio-dev \
    alsa-utils \
    libasound2-dev
```

### 2. Clone Audio Packages

```bash
cd ~/lucy_ws/src

# Clone audio_common (ROS2 compatible)
git clone https://github.com/mgonzs13/audio_common.git
```

### 3. Build Packages

```bash
cd ~/lucy_ws

# Build audio packages first
colcon build --packages-select audio_common_msgs

# Then build audio_common
colcon build --packages-select audio_common

# Finally build lucy_bringup
colcon build --packages-select lucy_bringup
```

### 4. Source Workspace

```bash
source ~/lucy_ws/install/setup.zsh
```

### 5. Verify Installation

```bash
# Check if nodes are available
ros2 run audio_common audio_capturer_node --help
ros2 run audio_common audio_player_node --help

# List available audio devices
arecord -l  # List capture devices
aplay -l    # List playback devices
```

---

## Configuration

### Launch Arguments

The audio system is configured through launch arguments in `lucy.launch.py`:

| Argument | Default | Description | Example |
|----------|---------|-------------|---------|
| `audio_sample_rate` | `48000` | Sample rate in Hz | `44100`, `48000` |
| `audio_capture_device` | `-1` | Capture device index (-1 = default) | `0`, `1`, `-1` |
| `audio_playback_device` | `-1` | Playback device index (-1 = default) | `0`, `1`, `-1` |

### Audio Capturer Parameters

```python
{
    'format': 8,           # paInt16 (PortAudio format constant)
    'channels': 2,          # Stereo microphones
    'rate': 48000,          # Sample rate (Hz)
    'chunk': 1024,          # Buffer size (frames)
    'device': -1,           # Device index (-1 = default)
    'frame_id': 'audio_capture'  # TF frame ID
}
```

### Audio Player Parameters

```python
{
    'channels': 2,          # Stereo speakers
    'device': -1            # Device index (-1 = default)
}
```

### PortAudio Format Constants

| Constant | Value | Description |
|----------|-------|-------------|
| `paFloat32` | 1 | 32-bit floating point |
| `paInt32` | 2 | 32-bit integer |
| `paInt16` | 8 | 16-bit integer (default) |
| `paInt8` | 16 | 8-bit integer |
| `paUInt8` | 32 | 8-bit unsigned integer |

**Note:** Current configuration uses `paInt16` (format: 8) for optimal performance on embedded systems.

### Finding Audio Device Indices

```bash
# List all audio capture devices
arecord -l

# Example output:
# card 0: PCH [HDA Intel PCH], device 0: ALC892 Analog [ALC892 Analog]
#   Subdevices: 1/1
#   Subdevice #0: subdevice #0
# 
# card 1: USB [USB Audio], device 0: USB Audio [USB Audio]
#   Subdevices: 1/1
#   Subdevice #0: subdevice #0

# List all audio playback devices
aplay -l

# Test a specific device
arecord -D hw:1,0 -d 5 test.wav  # Record from card 1, device 0
aplay -D hw:1,0 test.wav         # Play to card 1, device 0
```

**Device Index Mapping:**
- `-1` = Default system device
- `0` = First device (card 0, device 0)
- `1` = Second device (card 1, device 0)
- etc.

---

## Usage

### Basic Launch

Launch the entire Lucy system (includes audio):

```bash
source ~/lucy_ws/install/setup.zsh
ros2 launch lucy_bringup lucy.launch.py
```

### Custom Audio Configuration

```bash
# Launch with custom sample rate
ros2 launch lucy_bringup lucy.launch.py audio_sample_rate:=44100

# Launch with specific audio devices
ros2 launch lucy_bringup lucy.launch.py \
    audio_capture_device:=1 \
    audio_playback_device:=1

# Combined configuration
ros2 launch lucy_bringup lucy.launch.py \
    audio_sample_rate:=48000 \
    audio_capture_device:=0 \
    audio_playback_device:=0
```

### Standalone Audio Nodes

Run audio nodes independently for testing:

```bash
# Audio capturer only
ros2 run audio_common audio_capturer_node \
    --ros-args \
    -p format:=8 \
    -p channels:=2 \
    -p rate:=48000 \
    -p chunk:=1024 \
    -p device:=-1

# Audio player only
ros2 run audio_common audio_player_node \
    --ros-args \
    -p channels:=2 \
    -p device:=-1
```

### Using tmux Launcher

The recommended way to launch the full system:

```bash
~/launch_lucy.sh
```

This starts all nodes including audio in a tmux session.

### Monitoring Audio System

```bash
# Check if audio nodes are running
ros2 node list | grep audio

# Monitor audio topic
ros2 topic echo /audio

# Check audio topic frequency
ros2 topic hz /audio

# View audio topic info
ros2 topic info /audio

# Record audio to file (using ROS2 bag)
ros2 bag record /audio
```

---

## ROS2 Topics & Messages

### Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/audio` | `audio_common_msgs/AudioStamped` | Publisher: `audio_capturer_node`<br>Subscriber: `audio_player_node` | Audio data stream with timestamp |

### Message Structure

**`audio_common_msgs/AudioStamped`:**

```yaml
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
audio_common_msgs/Audio audio
  audio_common_msgs/AudioInformation info
    uint8 format          # PortAudio format constant
    uint8 channels       # Number of channels (2 for stereo)
    uint32 rate          # Sample rate (Hz)
    string coding_format # Audio coding format
  audio_common_msgs/AudioData audio_data
    float32[] float32_data  # For paFloat32 format
    int32[] int32_data      # For paInt32 format
    int16[] int16_data      # For paInt16 format (default)
    int8[] int8_data        # For paInt8 format
    uint8[] uint8_data      # For paUInt8 format
```

### Message Example

```python
# Example AudioStamped message (paInt16, stereo, 48kHz)
header:
  seq: 1234
  stamp:
    sec: 1701964800
    nanosec: 123456789
  frame_id: "audio_capture"
audio:
  info:
    format: 8          # paInt16
    channels: 2        # Stereo
    rate: 48000        # 48 kHz
    coding_format: "wave"
  audio_data:
    int16_data: [1234, -5678, 9012, -3456, ...]  # Interleaved stereo samples
```

**Data Format:**
- Audio samples are **interleaved** for stereo: `[L, R, L, R, L, R, ...]`
- For `paInt16`: Values range from -32768 to 32767
- Buffer size: 1024 frames = 2048 samples (for stereo)

---

## Node Details

### audio_capturer_node

**Purpose:** Captures audio from microphones and publishes to `/audio` topic.

**Parameters:**
- `format` (uint8): PortAudio format constant (default: 8 = paInt16)
- `channels` (uint8): Number of channels (default: 2 = stereo)
- `rate` (uint32): Sample rate in Hz (default: 48000)
- `chunk` (uint32): Buffer size in frames (default: 1024)
- `device` (int32): Audio device index (default: -1 = default device)
- `frame_id` (string): TF frame ID (default: "audio_capture")

**Published Topics:**
- `/audio` (`audio_common_msgs/AudioStamped`): Audio data stream

**Behavior:**
- Continuously captures audio from the specified device
- Publishes audio chunks at the configured sample rate
- Auto-respawns on failure (2 second delay)
- Handles device errors gracefully

### audio_player_node

**Purpose:** Subscribes to `/audio` topic and plays audio through speakers.

**Parameters:**
- `channels` (uint8): Number of channels (default: 2 = stereo)
- `device` (int32): Audio device index (default: -1 = default device)

**Subscribed Topics:**
- `/audio` (`audio_common_msgs/AudioStamped`): Audio data stream

**Behavior:**
- Continuously plays audio from the `/audio` topic
- Handles format conversion automatically
- Auto-respawns on failure (2 second delay)
- Reports underrun warnings when buffer is empty (normal when no audio is published)

---

## Troubleshooting

### Common Issues

#### 1. No Audio Input/Output

**Symptoms:**
- No audio data on `/audio` topic
- Audio player reports no data

**Solutions:**
```bash
# Check if audio devices are connected
arecord -l
aplay -l

# Test audio hardware directly
arecord -d 5 test.wav && aplay test.wav

# Check audio device permissions
ls -l /dev/snd/

# Verify ALSA configuration
alsamixer
```

#### 2. PortAudio Underrun Warnings

**Symptoms:**
```
PortAudio underrun detected, retrying...
```

**Explanation:**
These warnings are **normal and expected** when:
- No audio is being published to `/audio` topic
- Audio source stops temporarily
- System is idle

**Action:** No action needed. Warnings will stop when audio data starts flowing.

#### 3. ALSA Configuration Errors

**Symptoms:**
```
ALSA lib confmisc.c:1369:(snd_func_refer) Unable to find definition 'cards.0.pcm.front.0:CARD=0'
ALSA lib pcm.c:2664:(snd_pcm_open_noupdate) Unknown PCM front
```

**Explanation:**
These are ALSA configuration warnings, not errors. They occur when ALSA tries to use default device names that don't exist.

**Solutions:**
```bash
# Create/update ALSA configuration
sudo alsa force-reload

# Or ignore (system will use available devices)
```

#### 4. Device Not Found

**Symptoms:**
```
Error opening audio device
```

**Solutions:**
```bash
# List available devices
arecord -l
aplay -l

# Use correct device index in launch arguments
ros2 launch lucy_bringup lucy.launch.py \
    audio_capture_device:=0 \
    audio_playback_device:=0

# Test device directly
arecord -D hw:0,0 -d 5 test.wav
```

#### 5. High CPU Usage

**Symptoms:**
- System becomes sluggish
- Audio dropouts

**Solutions:**
```bash
# Reduce sample rate
ros2 launch lucy_bringup lucy.launch.py audio_sample_rate:=44100

# Increase buffer size (in code, modify chunk parameter)
# Default: 1024, try: 2048 or 4096

# Enable Jetson performance mode
sudo jetson_clocks
```

#### 6. Audio Nodes Not Starting

**Symptoms:**
- Nodes don't appear in `ros2 node list`
- Launch fails silently

**Solutions:**
```bash
# Check if audio_common is built
ros2 pkg list | grep audio_common

# Rebuild if needed
cd ~/lucy_ws
colcon build --packages-select audio_common

# Check node executables
ros2 run audio_common audio_capturer_node --help
ros2 run audio_common audio_player_node --help

# Check launch file syntax
python3 -m py_compile ~/lucy_ws/src/lucy_ros_packages/lucy_bringup/launch/lucy.launch.py
```

### Debug Commands

```bash
# Check audio nodes status
ros2 node list | grep audio
ros2 node info /audio_capturer
ros2 node info /audio_player

# Monitor audio topic
ros2 topic echo /audio --no-arr
ros2 topic hz /audio

# Check system audio
alsamixer                    # Audio mixer
speaker-test -t wav -c 2     # Test speakers
arecord -d 5 test.wav        # Test microphones

# View node logs
ros2 run rqt_console rqt_console
```

---

## Performance Optimization

### Sample Rate Selection

| Sample Rate | Use Case | CPU Usage | Quality |
|-------------|----------|-----------|---------|
| 44100 Hz | Standard audio, music | Low | Good |
| 48000 Hz | Professional audio, video sync | Medium | Excellent |
| 96000 Hz | High-fidelity recording | High | Superior |

**Recommendation:** Use 48000 Hz for balanced performance and quality on Jetson AGX Orin.

### Buffer Size Tuning

| Buffer Size | Latency | Stability | CPU Usage |
|-------------|---------|-----------|-----------|
| 512 frames | Low (~10ms) | Lower | Higher |
| 1024 frames | Medium (~21ms) | Good | Medium |
| 2048 frames | Higher (~42ms) | Excellent | Lower |

**Current Setting:** 1024 frames (optimal for real-time applications)

### Jetson-Specific Optimizations

```bash
# Enable maximum performance mode
sudo jetson_clocks

# Set CPU governor to performance
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# Disable power management throttling
sudo nvpmodel -m 0  # MAXN mode
```

### Memory Considerations

- **Audio buffer memory:** ~4 KB per 1024-frame chunk (stereo, 16-bit)
- **Topic queue:** Default 10 messages (~40 KB)
- **Total per node:** ~50 KB memory footprint

---

## Best Practices

### 1. Device Selection

- Always test devices before deployment
- Use device indices instead of names for reliability
- Document device mappings for your hardware setup

### 2. Sample Rate Consistency

- Use the same sample rate for capture and playback
- Match sample rate to your application requirements
- Consider downstream processing needs (speech recognition, etc.)

### 3. Error Handling

- Monitor node status regularly
- Check topic activity: `ros2 topic hz /audio`
- Implement health checks in your application

### 4. Resource Management

- Don't run unnecessary audio processing
- Use appropriate buffer sizes for your use case
- Monitor CPU usage during development

### 5. Testing

- Test audio hardware independently before ROS2 integration
- Verify audio quality with known test files
- Test with actual microphones and speakers, not just software loops

### 6. Documentation

- Document your specific hardware configuration
- Keep track of device indices for your setup
- Note any custom ALSA configurations

---

## API Reference

### Launch File Integration

The audio system is integrated into `lucy.launch.py` via the `create_audio_nodes()` helper function:

```python
def create_audio_nodes(sample_rate, capture_device, playback_device):
    """Create audio capture and playback nodes."""
    return [
        Node(
            package='audio_common',
            executable='audio_capturer_node',
            name='audio_capturer',
            # ... parameters ...
        ),
        Node(
            package='audio_common',
            executable='audio_player_node',
            name='audio_player',
            # ... parameters ...
        )
    ]
```

### Programmatic Usage

To use audio in your own ROS2 nodes:

```python
import rclpy
from rclpy.node import Node
from audio_common_msgs.msg import AudioStamped

class MyAudioProcessor(Node):
    def __init__(self):
        super().__init__('my_audio_processor')
        self.subscription = self.create_subscription(
            AudioStamped,
            '/audio',
            self.audio_callback,
            10
        )
    
    def audio_callback(self, msg):
        # Process audio data
        audio_data = msg.audio.audio_data.int16_data
        sample_rate = msg.audio.info.rate
        channels = msg.audio.info.channels
        # ... your processing ...
```

### Service Integration

Currently, audio nodes don't expose ROS2 services. For on-demand control, consider:
- Publishing to control topics
- Using lifecycle nodes
- Implementing custom service wrappers

---

## References

- **audio_common GitHub:** https://github.com/mgonzs13/audio_common
- **PortAudio Documentation:** http://www.portaudio.com/docs.html
- **ALSA Documentation:** https://www.alsa-project.org/wiki/Documentation
- **Jetson AGX Orin Audio:** https://developer.nvidia.com/embedded/jetson-agx-orin

---

## License

This documentation is part of the Lucy Robot project and is licensed under **GPL-3.0**.

**Copyright 2024 Lucy Robotics Team**

---

## Changelog

### Version 1.0.0 (December 2024)
- Initial comprehensive documentation
- Audio system integration with lucy_bringup
- Support for stereo capture and playback
- PortAudio-based implementation
- Jetson AGX Orin optimization

---

## Support

For issues, questions, or contributions:
- **Email:** contact@lucy-robotics.com
- **Repository:** [Lucy ROS Packages](https://github.com/sentience-robotics/lucy_ros_packages)

---

*Last updated: December 2024*


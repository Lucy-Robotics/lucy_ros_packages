#!/usr/bin/env python3
# Copyright 2024 Lucy Robotics Team
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32
from std_srvs.srv import SetBool

FPS = 10.0
CAMERA_DEVICE = '/dev/video0'


class CameraPublisher(Node):
    """
    Zero-copy MJPEG camera publisher using GStreamer pipeline.

    Publishes compressed JPEG images from camera with zero CPU transcoding overhead.
    Supports service-based streaming control (start_streaming/stop_streaming).
    Automatically activates when clients are detected and pauses when no clients are present.
    Listens to /lucy/client_count topic for automatic activation/deactivation.
    """

    def __init__(self):
        super().__init__('camera_publisher')

        self.declare_parameter('fps', FPS)
        self.declare_parameter('device', CAMERA_DEVICE)
        self.declare_parameter('vendor_id', '')
        self.declare_parameter('product_id', '')
        self.declare_parameter('serial_number', '')

        # Get parameter values
        self.target_fps = self.get_parameter('fps').get_parameter_value().double_value
        self.camera_device = self.get_parameter('device').get_parameter_value().string_value
        self.vendor_id = self.get_parameter('vendor_id').get_parameter_value().string_value
        self.product_id = self.get_parameter('product_id').get_parameter_value().string_value
        self.serial_number = self.get_parameter('serial_number').get_parameter_value().string_value

        # Find camera by vendor ID/serial if provided, or by default look for
        # "webcamproduct: usb-webcam". If no specific ID provided, try to find
        # the external webcam by name
        if self.vendor_id or self.product_id or self.serial_number:
            detected_device = self.find_camera_by_id(
                self.vendor_id, self.product_id, self.serial_number
            )
            if detected_device:
                self.camera_device = detected_device
                self.get_logger().info(f'Found camera by ID: {self.camera_device}')
            else:
                self.get_logger().warn(
                    f'Could not find camera by ID, using device path: '
                    f'{self.camera_device}'
                )
        else:
            # Default: try to find the external webcam by name
            detected_device = self.find_camera_by_id()
            if detected_device:
                self.camera_device = detected_device
                self.get_logger().info(
                    f'Found external camera (webcamproduct: usb-webcam): '
                    f'{self.camera_device}'
                )
            else:
                self.get_logger().info(f'Using default device path: {self.camera_device}')

        # Initialize camera hardware
        self.init_cap()

        # Create publisher
        self.jpg_pub = self.create_publisher(CompressedImage, 'ext_camera/jpg', 10)

        # Streaming state
        self.is_streaming = False
        self.client_count = 0
        self.last_publish_time = 0.0

        # Create timer for frame publishing (will be started/stopped by services)
        self.timer = None

        # Create services for streaming control
        self.create_service(SetBool, 'start_streaming', self.start_streaming_callback)
        self.create_service(SetBool, 'stop_streaming', self.stop_streaming_callback)
        # Subscribe to client count for automatic control
        self.create_subscription(Int32, '/lucy/client_count', self.client_count_callback, 10)

        self.get_logger().info(
            f'Camera publisher node started using device {self.camera_device} '
            f'at {self.target_fps} FPS'
        )

    def start_streaming_callback(self, request, response):
        """Service: Start streaming."""
        if not self.is_streaming:
            self.is_streaming = True
            # Start the timer if not already running
            if self.timer is None:
                self.timer = self.create_timer(0.01, self.publish_frame)  # 100Hz timer
            self.get_logger().info('Streaming started via service')
            response.success = True
            response.message = 'Streaming started'
        else:
            response.success = True
            response.message = 'Streaming already active'
        return response

    def stop_streaming_callback(self, request, response):
        """Service: Stop streaming."""
        if self.is_streaming:
            self.is_streaming = False
            if self.timer is not None:
                self.timer.cancel()
                self.timer = None
            self.get_logger().info('Streaming stopped via service')
        response.success = True
        response.message = 'Streaming stopped'
        return response

    def start_streaming_internal(self):
        """Start streaming internally."""
        if not self.is_streaming:
            self.is_streaming = True
            if self.timer is None:
                self.timer = self.create_timer(0.01, self.publish_frame)
            self.get_logger().info(
                f'{self.client_count} client(s) detected - starting camera '
                f'streaming on {self.camera_device}'
            )

    def stop_streaming_internal(self):
        """Stop streaming internally."""
        if self.is_streaming:
            self.is_streaming = False
            if self.timer is not None:
                self.timer.cancel()
                self.timer = None
            self.get_logger().info(
                f'No clients detected - stopping camera streaming on '
                f'{self.camera_device}'
            )

    def client_count_callback(self, msg):
        """Automatically start/stop streaming based on client count."""
        self.client_count = msg.data

        if msg.data > 0 and not self.is_streaming:
            self.start_streaming_internal()
        elif msg.data == 0 and self.is_streaming:
            self.stop_streaming_internal()

    def set_fps(self, fps):
        self.target_fps = max(1.0, min(30.0, fps))
        self.get_logger().info(f'Frame rate set to {self.target_fps} FPS')

    def find_camera_by_id(self, vendor_id=None, product_id=None, serial_number=None):
        """
        Find camera device by vendor ID, product ID, serial number, or by name.

        Uses v4l2-ctl to enumerate devices and match identifiers.
        Specifically looks for "webcamproduct: usb-webcam" to identify the
        external camera. Returns device path (e.g., /dev/video6) or None if
        not found.
        """
        import re
        import subprocess

        try:
            # List all video devices
            result = subprocess.run(
                ['v4l2-ctl', '--list-devices'],
                capture_output=True, text=True, timeout=5
            )
            # Check if stdout contains valid device information
            # v4l2-ctl may return non-zero if it can't open /dev/video0,
            # but still provide valid output for other devices
            if not result.stdout or 'video' not in result.stdout.lower():
                self.get_logger().warn('v4l2-ctl not available or no output, using device path')
                return None

            # Parse output to find matching device
            lines = result.stdout.split('\n')
            current_device_name = None
            current_devices = []

            for i, line in enumerate(lines):
                # Check if line contains device name (before device paths)
                if 'webcamproduct' in line.lower() or 'usb-webcam' in line.lower():
                    # Found the external webcam - extract name
                    current_device_name = line.strip()
                    current_devices = []
                    # Look ahead for device paths
                    for j in range(i + 1, min(i + 5, len(lines))):
                        if '/dev/video' in lines[j]:
                            match = re.search(r'/dev/video\d+', lines[j])
                            if match:
                                current_devices.append(match.group(0))
                    # Return the first video device (usually /dev/video6)
                    if current_devices:
                        self.get_logger().info(
                            f'Found external camera: {current_device_name} '
                            f'at {current_devices[0]}'
                        )
                        return current_devices[0]

                # Check if line contains device path
                elif '/dev/video' in line:
                    # Extract device path
                    match = re.search(r'/dev/video\d+', line)
                    if match:
                        # If we're in a section with a device name, associate it
                        if (current_device_name and
                                'webcamproduct' in current_device_name.lower()):
                            self.get_logger().info(
                                f'Found external camera device: {match.group(0)}'
                            )
                            return match.group(0)

            # If we have identifiers, try to match via sysfs
            if vendor_id or product_id or serial_number:
                try:
                    # Use sysfs to find matching device
                    import glob
                    import os

                    for video_dev in glob.glob('/dev/video*'):
                        dev_num = video_dev.replace('/dev/video', '')
                        sysfs_path = f'/sys/class/video4linux/video{dev_num}/device'

                        if os.path.exists(sysfs_path):
                            # Check vendor and product IDs
                            vendor_file = os.path.join(sysfs_path, 'idVendor')
                            product_file = os.path.join(sysfs_path, 'idProduct')

                            if os.path.exists(vendor_file) and vendor_id:
                                with open(vendor_file, 'r') as f:
                                    if vendor_id.lower() in f.read().lower():
                                        return video_dev

                            if os.path.exists(product_file) and product_id:
                                with open(product_file, 'r') as f:
                                    if product_id.lower() in f.read().lower():
                                        return video_dev
                except Exception as e:
                    self.get_logger().debug(f'Error matching device by ID: {e}')

            # If no match found, return None to use fallback device path
            return None

        except Exception as e:
            self.get_logger().warn(f'Error finding camera by ID: {e}')
            return None

    def init_cap(self):
        # Create GStreamer pipeline with parameterized device
        # Optimized for 1920x1080 at 30 FPS
        gst_pipeline = (
            f'v4l2src device={self.camera_device} ! '
            'image/jpeg,width=1920,height=1080,framerate=30/1 ! '
            'jpegparse ! '
            'appsink drop=true emit-signals=true sync=false'
        )

        self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

        if not self.cap.isOpened():
            self.get_logger().error(
                f'Failed to open camera on {self.camera_device} '
                f'with GStreamer pipeline'
            )
            return
        else:
            self.get_logger().info(f'Camera on {self.camera_device} opened successfully')

        # Optimize OpenCV settings for Jetson
        try:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        except Exception:
            pass

    def publish_frame(self):
        if not self.is_streaming:
            return

        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time - self.last_publish_time < (1.0 / self.target_fps):
            return

        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.get_logger().error(
                f'Frame read failed on {self.camera_device}, '
                f'reconnecting camera...'
            )
            self.cap.release()
            self.init_cap()
            return

        if frame.shape[0] == 1:  # Raw JPEG data
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = 'jpeg'
            msg.data = frame.flatten().tobytes()
            self.jpg_pub.publish(msg)

            self.last_publish_time = current_time
        else:
            self.get_logger().error(f'Unexpected frame shape: {frame.shape}')


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()

    rclpy.spin(node)


if __name__ == '__main__':
    main()

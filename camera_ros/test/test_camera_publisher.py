#!/usr/bin/env python3
# Copyright 2026 Lucy Robotics Team
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

import os
import sys
from unittest.mock import MagicMock
from unittest.mock import patch

import pytest
import rclpy
from std_msgs.msg import Int32
from std_srvs.srv import SetBool

# Add scripts directory to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))


@pytest.fixture
def rclpy_init_shutdown():
    """Fixture to initialize and shutdown rclpy for each test."""
    rclpy.init()
    yield
    rclpy.shutdown()


class TestCameraPublisher:
    """Unit tests for CameraPublisher node."""

    @patch('cv2.VideoCapture')
    @patch('subprocess.run')
    def test_camera_detection_webcamproduct(
        self, mock_subprocess, mock_videocapture, rclpy_init_shutdown
    ):
        """Test that camera is detected by webcamproduct name."""
        from camera_publisher import CameraPublisher

        # Mock v4l2-ctl output
        mock_subprocess.return_value.returncode = 0
        mock_subprocess.return_value.stdout = (
            """NVIDIA Tegra Video Input Device (platform:tegra-camrtc-ca):
        /dev/media0

webcamproduct: usb-webcam (usb-3610000.usb-4.1.2.2):
        /dev/video6
        /dev/video7
        /dev/media3
"""
        )

        # Mock VideoCapture
        mock_cap = MagicMock()
        mock_cap.isOpened.return_value = True
        mock_videocapture.return_value = mock_cap

        node = CameraPublisher()

        assert node.camera_device == '/dev/video6'
        mock_videocapture.assert_called_once()
        topic_name = node.jpg_pub.topic_name
        assert topic_name == '/ext_camera/jpg' or topic_name == 'ext_camera/jpg'

    @patch('cv2.VideoCapture')
    @patch('subprocess.run')
    def test_camera_detection_fallback(
        self, mock_subprocess, mock_videocapture, rclpy_init_shutdown
    ):
        """Test fallback to default device when camera not found."""
        from camera_publisher import CameraPublisher

        # Mock v4l2-ctl output without webcamproduct
        mock_subprocess.return_value.returncode = 0
        mock_subprocess.return_value.stdout = """NVIDIA Tegra Video Input Device:
        /dev/video0
"""

        # Mock VideoCapture
        mock_cap = MagicMock()
        mock_cap.isOpened.return_value = True
        mock_videocapture.return_value = mock_cap

        node = CameraPublisher()

        # Should use default device
        assert node.camera_device == '/dev/video0'

    @patch('cv2.VideoCapture')
    @patch('subprocess.run')
    def test_camera_detection_nonzero_exit_code(
        self, mock_subprocess, mock_videocapture, rclpy_init_shutdown
    ):
        """Test that camera detection works even when v4l2-ctl returns non-zero."""
        from camera_publisher import CameraPublisher

        # Mock v4l2-ctl with non-zero return code but valid stdout
        # This simulates the real behavior where v4l2-ctl returns 1 if it
        # can't open /dev/video0, but still lists available devices
        mock_subprocess.return_value.returncode = 1
        mock_subprocess.return_value.stdout = (
            """NVIDIA Tegra Video Input Device (platform:tegra-camrtc-ca):
        /dev/media0

webcamproduct: usb-webcam (usb-3610000.usb-4.1.2.2):
        /dev/video6
        /dev/video7
        /dev/media3
"""
        )

        # Mock VideoCapture
        mock_cap = MagicMock()
        mock_cap.isOpened.return_value = True
        mock_videocapture.return_value = mock_cap

        node = CameraPublisher()

        # Should still detect the camera despite non-zero return code
        assert node.camera_device == '/dev/video6'
        mock_videocapture.assert_called_once()

    @patch('cv2.VideoCapture')
    @patch('subprocess.run')
    def test_camera_detection_no_output(
        self, mock_subprocess, mock_videocapture, rclpy_init_shutdown
    ):
        """Test fallback when v4l2-ctl produces no output."""
        from camera_publisher import CameraPublisher

        # Mock v4l2-ctl with no output (truly unavailable)
        mock_subprocess.return_value.returncode = 1
        mock_subprocess.return_value.stdout = ''

        # Mock VideoCapture
        mock_cap = MagicMock()
        mock_cap.isOpened.return_value = True
        mock_videocapture.return_value = mock_cap

        node = CameraPublisher()

        # Should fall back to default device when no output
        assert node.camera_device == '/dev/video0'

    @patch('cv2.VideoCapture')
    @patch('subprocess.run')
    def test_start_streaming_service(
        self, mock_subprocess, mock_videocapture, rclpy_init_shutdown
    ):
        """Test start_streaming service."""
        from camera_publisher import CameraPublisher

        # Mock subprocess and VideoCapture
        mock_subprocess.return_value.returncode = 0
        mock_subprocess.return_value.stdout = (
            'webcamproduct: usb-webcam:\n        /dev/video6\n'
        )
        mock_cap = MagicMock()
        mock_cap.isOpened.return_value = True
        mock_videocapture.return_value = mock_cap

        node = CameraPublisher()
        node.is_streaming = False

        request = SetBool.Request()
        request.data = True
        response = node.start_streaming_callback(request, SetBool.Response())

        assert response.success is True
        assert node.is_streaming is True
        assert 'started' in response.message.lower()

    @patch('cv2.VideoCapture')
    @patch('subprocess.run')
    def test_stop_streaming_service(
        self, mock_subprocess, mock_videocapture, rclpy_init_shutdown
    ):
        """Test stop_streaming service."""
        from camera_publisher import CameraPublisher

        # Mock subprocess and VideoCapture
        mock_subprocess.return_value.returncode = 0
        mock_subprocess.return_value.stdout = (
            'webcamproduct: usb-webcam:\n        /dev/video6\n'
        )
        mock_cap = MagicMock()
        mock_cap.isOpened.return_value = True
        mock_videocapture.return_value = mock_cap

        node = CameraPublisher()
        node.is_streaming = True
        node.timer = MagicMock()

        request = SetBool.Request()
        request.data = True
        response = node.stop_streaming_callback(request, SetBool.Response())

        assert response.success is True
        assert node.is_streaming is False
        assert 'stopped' in response.message.lower()

    @patch('cv2.VideoCapture')
    @patch('subprocess.run')
    def test_client_count_callback_start(
        self, mock_subprocess, mock_videocapture, rclpy_init_shutdown
    ):
        """Test client count callback starts streaming."""
        from camera_publisher import CameraPublisher

        # Mock subprocess and VideoCapture
        mock_subprocess.return_value.returncode = 0
        mock_subprocess.return_value.stdout = (
            'webcamproduct: usb-webcam:\n        /dev/video6\n'
        )
        mock_cap = MagicMock()
        mock_cap.isOpened.return_value = True
        mock_videocapture.return_value = mock_cap

        node = CameraPublisher()
        node.is_streaming = False

        msg = Int32()
        msg.data = 1
        node.client_count_callback(msg)

        assert node.is_streaming is True
        assert node.client_count == 1

    @patch('cv2.VideoCapture')
    @patch('subprocess.run')
    def test_client_count_callback_stop(
        self, mock_subprocess, mock_videocapture, rclpy_init_shutdown
    ):
        """Test client count callback stops streaming."""
        from camera_publisher import CameraPublisher

        # Mock subprocess and VideoCapture
        mock_subprocess.return_value.returncode = 0
        mock_subprocess.return_value.stdout = (
            'webcamproduct: usb-webcam:\n        /dev/video6\n'
        )
        mock_cap = MagicMock()
        mock_cap.isOpened.return_value = True
        mock_videocapture.return_value = mock_cap

        node = CameraPublisher()
        node.is_streaming = True
        node.timer = MagicMock()

        msg = Int32()
        msg.data = 0
        node.client_count_callback(msg)

        assert node.is_streaming is False
        assert node.client_count == 0

    @patch('cv2.VideoCapture')
    @patch('subprocess.run')
    def test_topic_name_ext_camera(
        self, mock_subprocess, mock_videocapture, rclpy_init_shutdown
    ):
        """Test that topic name is ext_camera/jpg."""
        from camera_publisher import CameraPublisher

        # Mock subprocess and VideoCapture
        mock_subprocess.return_value.returncode = 0
        mock_subprocess.return_value.stdout = (
            'webcamproduct: usb-webcam:\n        /dev/video6\n'
        )
        mock_cap = MagicMock()
        mock_cap.isOpened.return_value = True
        mock_videocapture.return_value = mock_cap

        node = CameraPublisher()

        # Topic name may have leading slash
        topic_name = node.jpg_pub.topic_name
        assert topic_name == '/ext_camera/jpg' or topic_name == 'ext_camera/jpg'

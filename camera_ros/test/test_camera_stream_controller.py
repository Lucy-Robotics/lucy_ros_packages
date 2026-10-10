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
from unittest.mock import Mock
from unittest.mock import patch

import pytest
import rclpy
from std_msgs.msg import Int32

# Add scripts directory to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))


@pytest.fixture
def rclpy_init_shutdown():
    """Fixture to initialize and shutdown rclpy for each test."""
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def controller_node(rclpy_init_shutdown):
    """Create a CameraStreamController and destroy it before rclpy shuts down."""
    from camera_stream_controller import CameraStreamController
    node = CameraStreamController()
    yield node
    node.destroy_node()


class TestCameraStreamController:
    """Unit tests for CameraStreamController node."""

    def test_node_initialization(self, controller_node):
        """Test that controller node initializes correctly."""
        node = controller_node

        assert node.get_name() == 'camera_stream_controller'
        assert node.current_client_count == 0
        assert node.is_streaming is False
        assert node.start_streaming_client is not None
        assert node.stop_streaming_client is not None

    def test_client_count_callback_start(self, controller_node):
        """Test client count callback starts streaming when clients > 0."""
        from camera_stream_controller import CameraStreamController
        node = controller_node
        with patch.object(CameraStreamController, 'start_streaming') as mock_start:
            node.is_streaming = False

            msg = Int32()
            msg.data = 1
            node.client_count_callback(msg)

            mock_start.assert_called_once()
            assert node.current_client_count == 1

    def test_client_count_callback_stop(self, controller_node):
        """Test client count callback stops streaming when clients == 0."""
        from camera_stream_controller import CameraStreamController
        node = controller_node
        with patch.object(CameraStreamController, 'stop_streaming') as mock_stop:
            node.is_streaming = True

            msg = Int32()
            msg.data = 0
            node.client_count_callback(msg)

            mock_stop.assert_called_once()
            assert node.current_client_count == 0

    def test_start_streaming_service_available(self, controller_node):
        """Test start_streaming when service is available."""
        node = controller_node
        node.start_streaming_client.wait_for_service = Mock(return_value=True)
        node.start_streaming_client.call_async = Mock(return_value=MagicMock())

        node.start_streaming()

        node.start_streaming_client.wait_for_service.assert_called_once_with(
            timeout_sec=1.0
        )
        node.start_streaming_client.call_async.assert_called_once()

    def test_start_streaming_service_unavailable(self, controller_node):
        """Test start_streaming when service is unavailable."""
        node = controller_node
        node.start_streaming_client.wait_for_service = Mock(return_value=False)
        node.get_logger = Mock()

        node.start_streaming()

        node.start_streaming_client.wait_for_service.assert_called_once_with(
            timeout_sec=1.0
        )
        node.get_logger().warn.assert_called()

    def test_stop_streaming_service_available(self, controller_node):
        """Test stop_streaming when service is available."""
        node = controller_node
        node.stop_streaming_client.wait_for_service = Mock(return_value=True)
        node.stop_streaming_client.call_async = Mock(return_value=MagicMock())

        node.stop_streaming()

        node.stop_streaming_client.wait_for_service.assert_called_once_with(
            timeout_sec=1.0
        )
        node.stop_streaming_client.call_async.assert_called_once()

    def test_stop_streaming_service_unavailable(self, controller_node):
        """Test stop_streaming when service is unavailable."""
        node = controller_node
        node.stop_streaming_client.wait_for_service = Mock(return_value=False)
        node.get_logger = Mock()

        node.stop_streaming()

        node.stop_streaming_client.wait_for_service.assert_called_once_with(
            timeout_sec=1.0
        )
        node.get_logger().warn.assert_called()

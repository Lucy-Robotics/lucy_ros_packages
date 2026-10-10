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

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_srvs.srv import SetBool


class CameraStreamController(Node):
    """
    Controller node that manages camera streaming based on client count.

    Subscribes to /lucy/client_count topic and uses services to start/stop
    camera streaming when clients connect/disconnect.
    """

    def __init__(self):
        super().__init__('camera_stream_controller')

        # Subscribe to client count
        self.create_subscription(
            Int32, '/lucy/client_count', self.client_count_callback, 10
        )

        # Service clients for streaming control
        self.start_streaming_client = self.create_client(
            SetBool, '/camera_publisher/start_streaming'
        )
        self.stop_streaming_client = self.create_client(
            SetBool, '/camera_publisher/stop_streaming'
        )

        self.current_client_count = 0
        self.is_streaming = False

        self.get_logger().info('Camera stream controller started')

    def client_count_callback(self, msg):
        """Handle client count changes."""
        self.current_client_count = msg.data

        if msg.data > 0 and not self.is_streaming:
            self.start_streaming()
        elif msg.data == 0 and self.is_streaming:
            self.stop_streaming()

    def start_streaming(self):
        """Start streaming via service."""
        if not self.start_streaming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('start_streaming service not available')
            return

        request = SetBool.Request()
        request.data = True
        future = self.start_streaming_client.call_async(request)

        def response_callback(future):
            try:
                response = future.result()
                if response.success:
                    self.is_streaming = True
                    self.get_logger().info(
                        f'Camera streaming started '
                        f'({self.current_client_count} client(s))'
                    )
                else:
                    self.get_logger().warn(
                        f'Failed to start streaming: {response.message}'
                    )
            except Exception as e:
                self.get_logger().error(
                    f'Exception calling start_streaming service: {e}'
                )

        future.add_done_callback(response_callback)

    def stop_streaming(self):
        """Stop streaming via service."""
        if not self.stop_streaming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('stop_streaming service not available')
            return

        request = SetBool.Request()
        request.data = True
        future = self.stop_streaming_client.call_async(request)

        def response_callback(future):
            try:
                response = future.result()
                if response.success:
                    self.is_streaming = False
                    self.get_logger().info(
                        'Camera streaming stopped (no clients)'
                    )
                else:
                    self.get_logger().warn(
                        f'Failed to stop streaming: {response.message}'
                    )
            except Exception as e:
                self.get_logger().error(
                    f'Exception calling stop_streaming service: {e}'
                )

        future.add_done_callback(response_callback)


def main(args=None):
    rclpy.init(args=args)
    node = CameraStreamController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

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

"""Colorize a 16UC1 depth image (mm) into a JET JPEG for the control panel."""

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image


def colorize(depth_mm, min_mm, max_mm):
    """Map depth to JET (near = blue, far = red); invalid (0) pixels are black."""
    span = max(max_mm - min_mm, 1)
    scaled = (np.clip(depth_mm.astype(np.float32), min_mm, max_mm) - min_mm) * (
        255.0 / span
    )
    color = cv2.applyColorMap(scaled.astype(np.uint8), cv2.COLORMAP_JET)
    color[depth_mm == 0] = 0
    return color


class DepthColorizer(Node):
    def __init__(self):
        super().__init__("depth_colorizer")
        self.min_mm = self.declare_parameter("min_mm", 200).value
        self.max_mm = self.declare_parameter("max_mm", 3000).value
        self.quality = self.declare_parameter("jpeg_quality", 80).value
        self.bridge = CvBridge()
        self.pub = None
        self.sub = self.create_subscription(
            Image, "depth", self.on_depth, qos_profile_sensor_data
        )
        self.create_timer(1.0, self.follow_input)
        self.follow_input()

    def follow_input(self):
        """Advertise the output only while the camera publishes, so viewers see it go away."""
        has_input = self.count_publishers(self.sub.topic_name) > 0
        if has_input and self.pub is None:
            self.pub = self.create_publisher(
                CompressedImage, "depth/colorized/compressed", 10
            )
        elif not has_input and self.pub is not None:
            self.destroy_publisher(self.pub)
            self.pub = None

    def on_depth(self, msg):
        if self.pub is None or self.pub.get_subscription_count() == 0:
            return  # nobody watching: skip the encode
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        ok, jpg = cv2.imencode(
            ".jpg",
            colorize(depth, self.min_mm, self.max_mm),
            [cv2.IMWRITE_JPEG_QUALITY, self.quality],
        )
        if not ok:
            return
        out = CompressedImage(header=msg.header, format="jpeg")
        out.data = jpg.tobytes()
        self.pub.publish(out)


def main():
    rclpy.init()
    node = DepthColorizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

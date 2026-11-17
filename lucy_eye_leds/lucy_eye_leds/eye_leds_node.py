"""Republish the last eye LED colours latched, so a reloaded panel gets them back."""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSProfile
from std_msgs.msg import UInt32MultiArray

LATCHED = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


class EyeLedsNode(Node):

    def __init__(self):
        super().__init__("eye_leds")
        state = self.create_publisher(UInt32MultiArray, "/eye_leds/state", LATCHED)
        self.create_subscription(UInt32MultiArray, "/eye_leds", state.publish, 10)


def main():
    rclpy.init()
    node = EyeLedsNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

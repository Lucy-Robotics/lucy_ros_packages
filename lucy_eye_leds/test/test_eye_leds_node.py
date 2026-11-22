import time

from lucy_eye_leds.eye_leds_node import EyeLedsNode
from lucy_eye_leds.eye_leds_node import LATCHED
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import UInt32MultiArray

ISOLATED_DOMAIN = 87


@pytest.fixture
def executor():
    rclpy.init(domain_id=ISOLATED_DOMAIN)
    ex = SingleThreadedExecutor()
    yield ex
    for node in ex.get_nodes():
        node.destroy_node()
    rclpy.shutdown()


def spin_until(executor, done, timeout_s=5.0):
    deadline = time.monotonic() + timeout_s
    while not done() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
    return done()


def test_a_panel_connecting_after_the_colour_change_still_gets_it(executor):
    executor.add_node(EyeLedsNode())
    coach = rclpy.create_node("coach")
    executor.add_node(coach)
    command = coach.create_publisher(UInt32MultiArray, "/eye_leds", 10)
    assert spin_until(executor, lambda: command.get_subscription_count() > 0)
    command.publish(UInt32MultiArray(data=[0xFF0000, 0x00FF00]))
    spin_until(executor, lambda: False, timeout_s=0.3)

    panel = rclpy.create_node("panel")
    executor.add_node(panel)
    received = []
    panel.create_subscription(
        UInt32MultiArray,
        "/eye_leds/state",
        lambda msg: received.append(list(msg.data)),
        LATCHED,
    )
    assert spin_until(executor, lambda: received)
    assert received[-1] == [0xFF0000, 0x00FF00]

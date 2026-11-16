"""Publish the end-effector position and solve position IK for the web panel."""

from __future__ import annotations

from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .ik import configuration
from .ik import frame_position
from .ik import load_chain
from .ik import solve_position

END_EFFECTOR_TOPIC = "/lucy/end_effector"
TARGET_TOPIC = "/lucy/ik/target"
SOLUTION_TOPIC = "/lucy/ik/solution"
END_EFFECTOR_PERIOD_S = 0.1

_DESCRIPTION_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


class KinematicsNode(Node):
    """Kinematics of the end-effector frame of the robot on /robot_description."""

    def __init__(self) -> None:
        super().__init__("lucy_kinematics")
        self.declare_parameter("end_effector_frame", "end_effector")
        self._frame = str(self.get_parameter("end_effector_frame").value)
        self._chain = None
        self._positions: dict[str, float] = {}
        self._end_effector_pub = self.create_publisher(
            PointStamped, END_EFFECTOR_TOPIC, 10
        )
        self._solution_pub = self.create_publisher(JointState, SOLUTION_TOPIC, 10)
        self.create_subscription(
            String, "/robot_description", self._on_description, _DESCRIPTION_QOS
        )
        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)
        self.create_subscription(Point, TARGET_TOPIC, self._on_target, 10)
        self.create_timer(END_EFFECTOR_PERIOD_S, self._publish_end_effector)

    def _on_description(self, msg: String) -> None:
        self._chain = load_chain(msg.data, self._frame)
        if self._chain is None:
            self.get_logger().info(
                f"no {self._frame} frame in the robot; kinematics idle"
            )
        else:
            self.get_logger().info(
                f'{self._frame} moved by {", ".join(self._chain.joint_names)}'
            )

    def _on_joint_states(self, msg: JointState) -> None:
        self._positions.update(zip(msg.name, msg.position))

    def _publish_end_effector(self) -> None:
        if self._chain is None or not self._positions:
            return
        x, y, z = frame_position(
            self._chain, configuration(self._chain, self._positions)
        )
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._chain.root
        msg.point = Point(x=float(x), y=float(y), z=float(z))
        self._end_effector_pub.publish(msg)

    def _on_target(self, msg: Point) -> None:
        if self._chain is None or not self._positions:
            return
        q, _ = solve_position(
            self._chain,
            configuration(self._chain, self._positions),
            np.array([msg.x, msg.y, msg.z]),
        )
        solution = JointState()
        solution.header.stamp = self.get_clock().now().to_msg()
        solution.name = list(self._chain.joint_names)
        solution.position = [float(q[i]) for i in self._chain.q_indices]
        self._solution_pub.publish(solution)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KinematicsNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

"""Publish the end-effector position and pitch, and solve IK for the web panel."""

from __future__ import annotations

from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from lucy_msgs.msg import IkTarget
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_msgs.msg import String

from .ik import configuration
from .ik import frame_position
from .ik import load_chain
from .ik import solve_position
from .pitch import solve_pose
from .pitch import tool_pitch

END_EFFECTOR_TOPIC = "/lucy/end_effector"
TARGET_TOPIC = "/lucy/ik/target"
POSE_TARGET_TOPIC = "/lucy/ik/pose_target"
PITCH_TOPIC = "/lucy/end_effector_pitch"
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
        self._pitch_pub = self.create_publisher(Float64, PITCH_TOPIC, 10)
        self._solution_pub = self.create_publisher(JointState, SOLUTION_TOPIC, 10)
        self.create_subscription(
            String, "/robot_description", self._on_description, _DESCRIPTION_QOS
        )
        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)
        self.create_subscription(Point, TARGET_TOPIC, self._on_target, 10)
        self.create_subscription(IkTarget, POSE_TARGET_TOPIC, self._on_pose_target, 10)
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
        q = configuration(self._chain, self._positions)
        x, y, z = frame_position(self._chain, q)
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._chain.root
        msg.point = Point(x=float(x), y=float(y), z=float(z))
        self._end_effector_pub.publish(msg)
        pitch = tool_pitch(self._chain, q)
        if pitch is not None:
            self._pitch_pub.publish(Float64(data=pitch))

    def _on_target(self, msg: Point) -> None:
        if self._chain is None or not self._positions:
            return
        q, _ = solve_position(
            self._chain,
            configuration(self._chain, self._positions),
            np.array([msg.x, msg.y, msg.z]),
        )
        self._publish_solution(q, self._chain.joint_names)

    def _on_pose_target(self, msg: IkTarget) -> None:
        if self._chain is None or not self._positions:
            return
        q0 = configuration(self._chain, self._positions)
        target = np.array([msg.x, msg.y, msg.z])
        if tool_pitch(self._chain, q0) is None:
            q, _ = solve_position(self._chain, q0, target)
            self._publish_solution(q, self._chain.joint_names)
            return
        q, _, joints = solve_pose(self._chain, q0, target, msg.pitch)
        self._publish_solution(q, joints)

    def _publish_solution(self, q: np.ndarray, joints: list[str]) -> None:
        model = self._chain.model
        solution = JointState()
        solution.header.stamp = self.get_clock().now().to_msg()
        solution.name = list(joints)
        solution.position = [
            float(q[model.joints[model.getJointId(name)].idx_q]) for name in joints
        ]
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

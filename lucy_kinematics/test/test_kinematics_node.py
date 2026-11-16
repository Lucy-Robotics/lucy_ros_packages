from geometry_msgs.msg import Point
from lucy_kinematics.kinematics_node import KinematicsNode
import pytest
import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from test_ik import ARM


@pytest.fixture
def node():
    rclpy.init()
    n = KinematicsNode()
    n.published = []
    n._solution_pub.publish = n.published.append
    n._end_effector_pub.publish = n.published.append
    yield n
    n.destroy_node()
    rclpy.shutdown()


def test_target_is_answered_with_chain_joint_positions(node):
    node._on_description(String(data=ARM))
    node._on_joint_states(JointState(name=["shoulder", "elbow"], position=[0.0, 0.3]))
    node._on_target(Point(x=0.1, y=0.12, z=0.0))
    assert node.published[-1].name == ["shoulder", "elbow"]


def test_end_effector_position_is_published_in_the_root_frame(node):
    node._on_description(String(data=ARM))
    node._on_joint_states(JointState(name=["shoulder", "elbow"], position=[0.0, 0.0]))
    node._publish_end_effector()
    msg = node.published[-1]
    assert msg.header.frame_id == "base"
    assert (msg.point.x, msg.point.y) == pytest.approx((0.2, 0.0))


def test_robot_without_end_effector_stays_idle(node):
    node._on_description(String(data='<robot name="r"><link name="base"/></robot>'))
    node._on_joint_states(JointState(name=["a"], position=[0.0]))
    node._on_target(Point(x=0.1))
    node._publish_end_effector()
    assert node.published == []


def test_pose_target_is_answered_without_the_roll(node):
    from lucy_msgs.msg import IkTarget
    from test_pitch import WRIST_ARM

    node._on_description(String(data=WRIST_ARM))
    node._on_joint_states(
        JointState(
            name=["pan", "shoulder", "elbow", "wrist_pitch", "wrist_roll"],
            position=[0.0, 0.0, 0.4, 0.2, 0.7],
        )
    )
    node._on_pose_target(IkTarget(x=0.12, y=0.05, z=0.02, pitch=-0.6))
    assert node.published[-1].name == ["pan", "shoulder", "elbow", "wrist_pitch"]


def test_end_effector_pitch_is_published_with_its_position(node):
    from test_pitch import WRIST_ARM

    node._pitch_pub.publish = node.published.append
    node._on_description(String(data=WRIST_ARM))
    node._on_joint_states(JointState(name=["wrist_pitch"], position=[0.5]))
    node._publish_end_effector()
    assert node.published[-1].data == pytest.approx(-0.5)

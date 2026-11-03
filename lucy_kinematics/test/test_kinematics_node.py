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
    node._on_joint_states(JointState(name=['shoulder', 'elbow'], position=[0.0, 0.3]))
    node._on_target(Point(x=0.1, y=0.12, z=0.0))
    assert node.published[-1].name == ['shoulder', 'elbow']


def test_end_effector_position_is_published_in_the_root_frame(node):
    node._on_description(String(data=ARM))
    node._on_joint_states(JointState(name=['shoulder', 'elbow'], position=[0.0, 0.0]))
    node._publish_end_effector()
    msg = node.published[-1]
    assert msg.header.frame_id == 'base'
    assert (msg.point.x, msg.point.y) == pytest.approx((0.2, 0.0))


def test_robot_without_end_effector_stays_idle(node):
    node._on_description(String(data='<robot name="r"><link name="base"/></robot>'))
    node._on_joint_states(JointState(name=['a'], position=[0.0]))
    node._on_target(Point(x=0.1))
    node._publish_end_effector()
    assert node.published == []

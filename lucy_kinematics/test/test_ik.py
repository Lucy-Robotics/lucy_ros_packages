from lucy_kinematics.ik import configuration, frame_position, load_chain, solve_position
import numpy as np
import pytest

ARM = """
<robot name="arm">
  <link name="base"/>
  <link name="upper"/>
  <link name="lower"/>
  <link name="finger"/>
  <link name="end_effector"/>
  <joint name="shoulder" type="revolute">
    <parent link="base"/><child link="upper"/><axis xyz="0 0 1"/>
    <limit lower="-3" upper="3" effort="1" velocity="1"/>
  </joint>
  <joint name="elbow" type="revolute">
    <parent link="upper"/><child link="lower"/><origin xyz="0.1 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3" upper="3" effort="1" velocity="1"/>
  </joint>
  <joint name="grip" type="revolute">
    <parent link="lower"/><child link="finger"/><origin xyz="0.1 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <joint name="tip" type="fixed">
    <parent link="lower"/><child link="end_effector"/><origin xyz="0.1 0 0"/>
  </joint>
</robot>
"""


@pytest.fixture
def chain():
    return load_chain(ARM, "end_effector")


def test_chain_holds_only_the_joints_that_move_the_frame(chain):
    assert chain.joint_names == ["shoulder", "elbow"]
    assert chain.root == "base"


def test_missing_frame_gives_no_chain():
    assert load_chain(ARM, "gripper_tip") is None


def test_forward_kinematics_of_the_straight_arm(chain):
    q = configuration(chain, {"shoulder": 0.0, "elbow": 0.0})
    assert np.allclose(frame_position(chain, q), [0.2, 0.0, 0.0])


def test_reachable_target_is_reached(chain):
    target = np.array([0.1, 0.12, 0.0])
    q, error = solve_position(chain, configuration(chain, {"elbow": 0.3}), target)
    assert error < 1e-3
    assert np.allclose(frame_position(chain, q), target, atol=1e-3)


def test_unreachable_target_stops_at_the_closest_pose_within_limits(chain):
    q, error = solve_position(
        chain, configuration(chain, {}), np.array([1.0, 0.0, 0.0])
    )
    assert error == pytest.approx(0.8, abs=1e-2)
    assert np.all(np.abs(q[chain.q_indices]) <= 3.0)

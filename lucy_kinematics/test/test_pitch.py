from lucy_kinematics.ik import configuration, frame_position, load_chain
from lucy_kinematics.pitch import pitch_joints, solve_pose, tool_pitch
import numpy as np
import pytest

WRIST_ARM = """
<robot name="wrist_arm">
  <link name="base"/>
  <link name="turret"/>
  <link name="upper"/>
  <link name="lower"/>
  <link name="wrist"/>
  <link name="gripper"/>
  <link name="end_effector"/>
  <joint name="pan" type="revolute">
    <parent link="base"/><child link="turret"/><axis xyz="0 0 1"/>
    <limit lower="-3" upper="3" effort="1" velocity="1"/>
  </joint>
  <joint name="shoulder" type="revolute">
    <parent link="turret"/><child link="upper"/><origin xyz="0 0 0.05"/><axis xyz="0 1 0"/>
    <limit lower="-3" upper="3" effort="1" velocity="1"/>
  </joint>
  <joint name="elbow" type="revolute">
    <parent link="upper"/><child link="lower"/><origin xyz="0.1 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-3" upper="3" effort="1" velocity="1"/>
  </joint>
  <joint name="wrist_pitch" type="revolute">
    <parent link="lower"/><child link="wrist"/><origin xyz="0.1 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-3" upper="3" effort="1" velocity="1"/>
  </joint>
  <joint name="wrist_roll" type="revolute">
    <parent link="wrist"/><child link="gripper"/><origin xyz="0.02 0 0"/><axis xyz="1 0 0"/>
    <limit lower="-3" upper="3" effort="1" velocity="1"/>
  </joint>
  <joint name="tip" type="fixed">
    <parent link="gripper"/><child link="end_effector"/><origin xyz="0.05 0 0"/>
  </joint>
</robot>
"""


@pytest.fixture
def chain():
    return load_chain(WRIST_ARM, "end_effector")


def test_a_straight_arm_holds_the_tool_level(chain):
    assert tool_pitch(chain, configuration(chain, {})) == pytest.approx(0.0)


def test_tilting_the_wrist_down_lowers_the_tool_pitch(chain):
    q = configuration(chain, {"wrist_pitch": 0.5})
    assert tool_pitch(chain, q) == pytest.approx(-0.5)


def test_rolling_the_wrist_leaves_the_pitch_alone(chain):
    q = configuration(chain, {"wrist_pitch": 0.5, "wrist_roll": 1.2})
    assert tool_pitch(chain, q) == pytest.approx(-0.5)


def test_the_roll_joint_is_left_out_of_the_pitch_solve(chain):
    assert pitch_joints(chain, configuration(chain, {})) == [
        "pan",
        "shoulder",
        "elbow",
        "wrist_pitch",
    ]


def test_a_reachable_pose_is_reached_with_its_pitch(chain):
    q0 = configuration(chain, {"elbow": 0.4, "wrist_pitch": 0.2})
    target = np.array([0.12, 0.05, 0.02])
    q, error, _ = solve_pose(chain, q0, target, -0.6)
    assert error < 1e-3
    assert np.allclose(frame_position(chain, q), target, atol=1e-3)
    assert tool_pitch(chain, q) == pytest.approx(-0.6, abs=1e-2)


def test_the_pose_solve_never_turns_the_roll(chain):
    q0 = configuration(chain, {"elbow": 0.4, "wrist_roll": 0.7})
    q, _, joints = solve_pose(chain, q0, np.array([0.12, 0.05, 0.02]), -0.6)
    roll = chain.model.joints[chain.model.getJointId("wrist_roll")].idx_q
    assert "wrist_roll" not in joints
    assert q[roll] == pytest.approx(0.7)


def test_an_arm_whose_frame_sits_on_its_last_joint_has_no_pitch():
    flat = WRIST_ARM.replace('<origin xyz="0.05 0 0"/>', '<origin xyz="0 0 0"/>')
    chain = load_chain(flat, "end_effector")
    assert tool_pitch(chain, configuration(chain, {})) is None

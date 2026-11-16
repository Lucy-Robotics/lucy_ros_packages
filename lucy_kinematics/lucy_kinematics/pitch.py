"""Tool pitch: the angle above horizontal of the axis from the last joint to the frame."""

from __future__ import annotations

import numpy as np
import pinocchio as pin

from .ik import Chain
from .ik import DAMPING
from .ik import frame_position
from .ik import MAX_ITERATIONS
from .ik import STEP
from .ik import TOLERANCE_M

PITCH_WEIGHT_M = 0.1
ROLL_ALIGNMENT = 0.95
DIFF_STEP = 1e-6


def _joint_id(chain: Chain, name: str) -> int:
    return chain.model.getJointId(name)


def _tool_axis(chain: Chain, q: np.ndarray) -> np.ndarray:
    tip = frame_position(chain, q)
    last = chain.data.oMi[_joint_id(chain, chain.joint_names[-1])].translation
    return tip - last


def tool_pitch(chain: Chain, q: np.ndarray) -> float | None:
    """Pitch in radians, or None when the frame sits on the last joint."""
    axis = _tool_axis(chain, q)
    if np.linalg.norm(axis) < 1e-6:
        return None
    return float(np.arctan2(axis[2], np.hypot(axis[0], axis[1])))


def pitch_joints(chain: Chain, q: np.ndarray) -> list[str]:
    """Chain joints that pitch may move: a last joint turning about the tool axis is roll."""
    axis = _tool_axis(chain, q)
    last = chain.model.joints[_joint_id(chain, chain.joint_names[-1])].idx_v
    turning = pin.computeFrameJacobian(
        chain.model, chain.data, q, chain.frame_id, pin.LOCAL_WORLD_ALIGNED
    )[3:, last]
    alignment = abs(turning @ axis) / max(
        np.linalg.norm(turning) * np.linalg.norm(axis), 1e-12
    )
    if alignment > ROLL_ALIGNMENT:
        return chain.joint_names[:-1]
    return list(chain.joint_names)


def _pitch_row(chain: Chain, q: np.ndarray, q_indices: list[int]) -> np.ndarray:
    here = tool_pitch(chain, q)
    row = np.zeros(len(q_indices))
    for column, index in enumerate(q_indices):
        moved = q.copy()
        moved[index] += DIFF_STEP
        row[column] = (tool_pitch(chain, moved) - here) / DIFF_STEP
    return row


def solve_pose(
    chain: Chain, q0: np.ndarray, target: np.ndarray, pitch: float
) -> tuple[np.ndarray, float, list[str]]:
    """Bring the frame to ``target`` with the tool at ``pitch``; return q, error, moved joints."""
    model, data = chain.model, chain.data
    q = q0.copy()
    joints = pitch_joints(chain, q)
    ids = [model.joints[_joint_id(chain, name)] for name in joints]
    q_indices = [joint.idx_q for joint in ids]
    columns = [joint.idx_v for joint in ids]
    lower = model.lowerPositionLimit[q_indices]
    upper = model.upperPositionLimit[q_indices]

    def error() -> np.ndarray:
        position = target - frame_position(chain, q)
        return np.append(position, PITCH_WEIGHT_M * (pitch - tool_pitch(chain, q)))

    current = error()
    for _ in range(MAX_ITERATIONS):
        if np.linalg.norm(current) < TOLERANCE_M:
            break
        jacobian = np.vstack(
            [
                pin.computeFrameJacobian(
                    model, data, q, chain.frame_id, pin.LOCAL_WORLD_ALIGNED
                )[:3, columns],
                PITCH_WEIGHT_M * _pitch_row(chain, q, q_indices),
            ]
        )
        step = jacobian.T @ np.linalg.solve(
            jacobian @ jacobian.T + DAMPING * np.eye(4), current
        )
        q[q_indices] = np.clip(q[q_indices] + STEP * step, lower, upper)
        current = error()
    return q, float(np.linalg.norm(current)), joints

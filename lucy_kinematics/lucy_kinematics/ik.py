"""Position inverse kinematics of one frame with Pinocchio (damped least squares)."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pinocchio as pin

MAX_ITERATIONS = 200
TOLERANCE_M = 1e-4
DAMPING = 1e-4
STEP = 0.5


@dataclass
class Chain:
    """Model of a robot reduced to the joints that move one frame."""

    model: pin.Model
    data: pin.Data
    frame_id: int
    root: str
    joint_names: list[str]
    q_indices: list[int]


def load_chain(urdf: str, frame: str) -> Chain | None:
    """Build the chain moving ``frame``, or None when the URDF has no such frame."""
    model = pin.buildModelFromXML(urdf)
    if not model.existFrame(frame):
        return None
    frame_id = model.getFrameId(frame)
    support = model.supports[model.frames[frame_id].parentJoint]
    joints = [j for j in support if j != 0 and model.joints[j].nq == 1]
    return Chain(
        model=model,
        data=model.createData(),
        frame_id=frame_id,
        root=next(f.name for f in model.frames if f.type == pin.FrameType.BODY),
        joint_names=[model.names[j] for j in joints],
        q_indices=[model.joints[j].idx_q for j in joints],
    )


def configuration(chain: Chain, positions: dict[str, float]) -> np.ndarray:
    """Full configuration from named joint positions; unknown joints stay neutral."""
    q = pin.neutral(chain.model)
    for name, value in positions.items():
        if chain.model.existJointName(name):
            joint = chain.model.joints[chain.model.getJointId(name)]
            if joint.nq == 1:
                q[joint.idx_q] = value
    return q


def frame_position(chain: Chain, q: np.ndarray) -> np.ndarray:
    """World position of the chain's frame at configuration ``q``."""
    pin.framesForwardKinematics(chain.model, chain.data, q)
    return chain.data.oMf[chain.frame_id].translation.copy()


def solve_position(
    chain: Chain, q0: np.ndarray, target: np.ndarray
) -> tuple[np.ndarray, float]:
    """Move only the chain's joints to bring the frame towards ``target``; return q and error."""
    model, data = chain.model, chain.data
    q = q0.copy()
    lower = model.lowerPositionLimit[chain.q_indices]
    upper = model.upperPositionLimit[chain.q_indices]
    columns = [model.joints[model.getJointId(n)].idx_v for n in chain.joint_names]
    error = target - frame_position(chain, q)
    for _ in range(MAX_ITERATIONS):
        if np.linalg.norm(error) < TOLERANCE_M:
            break
        jacobian = pin.computeFrameJacobian(
            model, data, q, chain.frame_id, pin.LOCAL_WORLD_ALIGNED
        )[:3, columns]
        step = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + DAMPING * np.eye(3), error)
        q[chain.q_indices] = np.clip(q[chain.q_indices] + STEP * step, lower, upper)
        error = target - frame_position(chain, q)
    return q, float(np.linalg.norm(error))

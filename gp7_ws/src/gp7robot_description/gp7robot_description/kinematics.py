"""GP7 forward and inverse kinematics from the supplied URDF joint frames.

The DH frames use the standard convention

    A_i = Rz(theta_i) Tz(d_i) Tx(a_i) Rx(alpha_i).

The table is derived from the six revolute axes and origins in ``GP7.urdf``.
The final fixed rotation maps the constructed DH tool frame to the URDF's
``Link6`` frame, so poses returned by :func:`forward_kinematics` match the CAD
model frame used by the Gazebo robot.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

import numpy as np


JOINT_NAMES = ("joint1", "Joint2", "Joint3", "Join4", "Join5", "Joint6")


@dataclass(frozen=True)
class DHRow:
    joint: str
    theta_offset: float  # radians
    d: float  # metres
    a: float  # metres
    alpha: float  # radians


DH_TABLE = (
    DHRow("S", math.radians(-112.650759799361), 0.330000000000, 0.040000000186, math.radians(-90.0)),
    DHRow("L", math.radians(-104.091530693979), 0.000000000000, 0.445000000000, math.radians(0.0)),
    DHRow("U", math.radians(174.798565581791), 0.000000000000, 0.039999999934, math.radians(-90.0)),
    DHRow("R", math.radians(-9.999933172264), 0.440000000001, 0.000000000000, math.radians(90.0)),
    DHRow("B", math.radians(-177.345549555221), 0.000000000000, 0.000000000000, math.radians(-90.0)),
    DHRow("T", math.radians(0.0), -0.058893322297, 0.000000000000, math.radians(0.0)),
)

# Limits in the CAD joint-zero convention.
#  * Joint3 (U): the SolidWorks export had this range mirrored
#    ([-255.2, +4.8] deg). That range contains the pose at about -75 deg where
#    the forearm folds back onto the upper arm (wrist inside the shoulder),
#    which a real GP7 cannot do. The mirrored range [-4.8, +255.2] keeps the
#    datasheet's 260 deg span (U -70/+190) and excludes that fold.
#  * Joint4 (R): the export limited this to [-20, 0] deg; the datasheet gives
#    +/-190 deg.
JOINT_LIMITS = np.array(
    [
        [-1.76541379939274, 4.16870565738798],
        [-1.38040762104709, 1.84845149514013],
        [-0.0837507687984118, 4.45410528641014],
        [-3.31612557878923, 3.31612557878923],
        [-2.40252339028479, 2.30986559009995],
        [-2.42328686755966, 3.85640778111595],
    ],
    dtype=float,
)

# Joint maximum speeds from the GP7 column of the supplied GP7/GP8 datasheet.
MAX_JOINT_SPEEDS = np.deg2rad(np.array([375.0, 315.0, 410.0, 550.0, 550.0, 1000.0]))

# Conservative joint acceleration limits (rad/s^2) used for planned moves.
ACCELERATION_LIMITS = np.array([0.45, 0.45, 0.55, 0.80, 0.80, 1.00])

# Reference pose that defines the gripper mount and the "upright" Link6
# orientation. In this pose the flange points straight down above Box B
# (flange at 0.35, 0.25, 0.42 m), so the gripper is mounted coaxially with the
# joint-6 axis: rotating joint 6 only spins the gripper instead of swinging it
# into Link5. The pick/place solver also seeds IK from this pose.
UPRIGHT_REFERENCE_CONFIGURATION = np.array(
    [
        2.58637615,
        0.37832553,
        3.91332751,
        0.17453176,
        0.71113281,
        0.0,
    ],
    dtype=float,
)

# Rotation from the DH-defined wrist-axis frame to the CAD Link6 frame. The
# translation is zero because the DH d6 term ends at the Link6 origin.
DH_TO_LINK6_ROTATION = np.array(
    [
        [0.0, 0.75406926, -0.65679491],
        [0.0, -0.65679491, -0.75406926],
        [-1.0, 0.0, 0.0],
    ],
    dtype=float,
)


def _dh_transform(row: DHRow, joint_angle: float) -> np.ndarray:
    theta = row.theta_offset + joint_angle
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(row.alpha), math.sin(row.alpha)
    return np.array(
        [
            [ct, -st * ca, st * sa, row.a * ct],
            [st, ct * ca, -ct * sa, row.a * st],
            [0.0, sa, ca, row.d],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def forward_kinematics(joint_positions: Iterable[float]) -> np.ndarray:
    """Return the 4x4 base-to-Link6 transform for six joint angles in radians."""
    q = np.asarray(tuple(joint_positions), dtype=float)
    if q.shape != (6,):
        raise ValueError("GP7 forward kinematics requires six joint angles")
    transform = np.eye(4)
    for row, angle in zip(DH_TABLE, q):
        transform = transform @ _dh_transform(row, float(angle))
    transform[:3, :3] = transform[:3, :3] @ DH_TO_LINK6_ROTATION
    return transform


def geometric_jacobian(joint_positions: Iterable[float]) -> np.ndarray:
    """Return the 6x6 geometric Jacobian in the base frame."""
    q = np.asarray(tuple(joint_positions), dtype=float)
    if q.shape != (6,):
        raise ValueError("GP7 Jacobian requires six joint angles")

    transform = np.eye(4)
    origins: list[np.ndarray] = []
    axes: list[np.ndarray] = []
    for row, angle in zip(DH_TABLE, q):
        origins.append(transform[:3, 3].copy())
        axes.append(transform[:3, 2].copy())
        transform = transform @ _dh_transform(row, float(angle))

    tool_position = transform[:3, 3]
    jacobian = np.zeros((6, 6), dtype=float)
    for index, (origin, axis) in enumerate(zip(origins, axes)):
        jacobian[:3, index] = np.cross(axis, tool_position - origin)
        jacobian[3:, index] = axis
    return jacobian


def _rotation_vector(rotation: np.ndarray) -> np.ndarray:
    """Convert a rotation matrix to its axis-angle vector."""
    cosine = float(np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0))
    angle = math.acos(cosine)
    skew = np.array(
        [
            rotation[2, 1] - rotation[1, 2],
            rotation[0, 2] - rotation[2, 0],
            rotation[1, 0] - rotation[0, 1],
        ],
        dtype=float,
    )
    if angle < 1.0e-8:
        return 0.5 * skew
    sine = math.sin(angle)
    if abs(sine) < 1.0e-8:
        # The near-pi case is rare for the short pick-and-place moves. This
        # stable diagonal extraction avoids division by a tiny sine.
        axis = np.sqrt(np.maximum((np.diag(rotation) + 1.0) * 0.5, 0.0))
        largest = int(np.argmax(axis))
        if largest == 0 and axis[0] > 1e-8:
            axis[1] = math.copysign(axis[1], rotation[0, 1] + rotation[1, 0])
            axis[2] = math.copysign(axis[2], rotation[0, 2] + rotation[2, 0])
        elif largest == 1 and axis[1] > 1e-8:
            axis[0] = math.copysign(axis[0], rotation[0, 1] + rotation[1, 0])
            axis[2] = math.copysign(axis[2], rotation[1, 2] + rotation[2, 1])
        elif axis[2] > 1e-8:
            axis[0] = math.copysign(axis[0], rotation[0, 2] + rotation[2, 0])
            axis[1] = math.copysign(axis[1], rotation[1, 2] + rotation[2, 1])
        norm = np.linalg.norm(axis)
        return angle * axis / norm if norm > 0.0 else np.zeros(3)
    return (angle / (2.0 * sine)) * skew


def inverse_kinematics(
    target: np.ndarray,
    seed: Iterable[float] | None = None,
    *,
    position_tolerance: float = 0.003,
    orientation_tolerance: float = math.radians(2.0),
    max_iterations: int = 250,
) -> np.ndarray:
    """Solve a pose target with a limit-aware damped least-squares iteration.

    ``target`` is a base-to-Link6 4x4 transform. The returned angles use the
    CAD URDF joint zero conventions and are constrained to the exported joint
    limits.
    """
    target = np.asarray(target, dtype=float)
    if target.shape != (4, 4):
        raise ValueError("IK target must be a 4x4 homogeneous transform")
    q = np.zeros(6, dtype=float) if seed is None else np.asarray(tuple(seed), dtype=float).copy()
    if q.shape != (6,):
        raise ValueError("IK seed must contain six joint angles")
    q = np.clip(q, JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])

    damping = 0.035
    best_error = math.inf
    best_q = q.copy()
    for _ in range(max_iterations):
        current = forward_kinematics(q)
        position_error = target[:3, 3] - current[:3, 3]
        orientation_error = _rotation_vector(target[:3, :3] @ current[:3, :3].T)
        error = np.concatenate((position_error, orientation_error))
        position_norm = float(np.linalg.norm(position_error))
        orientation_norm = float(np.linalg.norm(orientation_error))
        score = position_norm + 0.20 * orientation_norm
        if score < best_error:
            best_error, best_q = score, q.copy()
        if position_norm <= position_tolerance and orientation_norm <= orientation_tolerance:
            return q

        jacobian = geometric_jacobian(q)
        normal = jacobian @ jacobian.T + damping * damping * np.eye(6)
        try:
            delta = jacobian.T @ np.linalg.solve(normal, error)
        except np.linalg.LinAlgError:
            damping *= 4.0
            continue
        maximum_step = float(np.max(np.abs(delta)))
        if maximum_step > 0.20:
            delta *= 0.20 / maximum_step

        candidate = np.clip(q + delta, JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])
        candidate_pose = forward_kinematics(candidate)
        candidate_error = np.concatenate(
            (
                target[:3, 3] - candidate_pose[:3, 3],
                _rotation_vector(target[:3, :3] @ candidate_pose[:3, :3].T),
            )
        )
        candidate_score = float(np.linalg.norm(candidate_error[:3])) + 0.20 * float(
            np.linalg.norm(candidate_error[3:])
        )
        if candidate_score < score:
            q = candidate
            damping = max(0.008, damping * 0.75)
        else:
            damping = min(0.4, damping * 2.0)

    best_pose = forward_kinematics(best_q)
    position_error = float(np.linalg.norm(target[:3, 3] - best_pose[:3, 3]))
    orientation_error = float(
        np.linalg.norm(_rotation_vector(target[:3, :3] @ best_pose[:3, :3].T))
    )
    raise ValueError(
        "No IK solution within tolerance: "
        f"position error {position_error:.4f} m, orientation error "
        f"{math.degrees(orientation_error):.2f} deg"
    )


def pose(position: Iterable[float], rotation: np.ndarray | None = None) -> np.ndarray:
    """Build a homogeneous pose from a 3-vector and a 3x3 rotation."""
    xyz = np.asarray(tuple(position), dtype=float)
    if xyz.shape != (3,):
        raise ValueError("Pose position must contain x, y, z")
    result = np.eye(4)
    result[:3, :3] = np.eye(3) if rotation is None else np.asarray(rotation, dtype=float)
    result[:3, 3] = xyz
    return result


def horizontal_reach(position: Iterable[float]) -> float:
    """Return base-plane radius of a target point in metres."""
    xyz = np.asarray(tuple(position), dtype=float)
    return float(math.hypot(float(xyz[0]), float(xyz[1])))


"""Frames and transforms.

    +X -> behind the rider      +Y -> rider's RIGHT      -Y -> rider's LEFT      +Z -> up

Frame tree:  radar (root) -> imu (fixed extrinsic T_radar_imu)
                          -> level (time-varying T_level_radar: roll/pitch removed, yaw retained)
There is no world, odom or bike frame.

Naming: ``T_target_source`` maps a point from ``source`` into ``target``.
Composition is right-to-left: ``p_radar = T_radar_imu @ p_imu``.
Quaternions are stored as [w, x, y, z]; rotation matrices are used for math;
Euler angles (intrinsic Z-Y-X) exist for display only.
"""
from __future__ import annotations

import math
from typing import Sequence, Tuple

import numpy as np

from .types import SourceTag, Transform

Quat = Tuple[float, float, float, float]


class FrameMismatchError(ValueError):
    """Raised when transforms are composed across mismatched frame names."""


# --- quaternion algebra ([w, x, y, z]) ---------------------------------------------
def q_normalize(q: Sequence[float]) -> np.ndarray:
    a = np.asarray(q, dtype=float)
    n = np.linalg.norm(a)
    if n == 0.0:
        raise ValueError("zero quaternion")
    return a / n


def q_mul(a: Sequence[float], b: Sequence[float]) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def q_conj(q: Sequence[float]) -> np.ndarray:
    w, x, y, z = q
    return np.array([w, -x, -y, -z])


def q_to_matrix(q: Sequence[float]) -> np.ndarray:
    w, x, y, z = q_normalize(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_q(R: np.ndarray) -> np.ndarray:
    R = np.asarray(R, dtype=float)
    tr = np.trace(R)
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([w, x, y, z])
    if q[0] < 0:
        q = -q
    return q_normalize(q)


def q_from_rotvec(rv: Sequence[float]) -> np.ndarray:
    v = np.asarray(rv, dtype=float)
    ang = np.linalg.norm(v)
    if ang < 1e-12:
        return np.array([1.0, 0.5 * v[0], 0.5 * v[1], 0.5 * v[2]]) / math.sqrt(1 + 0.25 * ang * ang)
    axis = v / ang
    s = math.sin(ang / 2)
    return np.array([math.cos(ang / 2), axis[0] * s, axis[1] * s, axis[2] * s])


def q_rotate(q: Sequence[float], v: Sequence[float]) -> np.ndarray:
    return q_to_matrix(q) @ np.asarray(v, dtype=float)


def q_slerp(q0: Sequence[float], q1: Sequence[float], u: float) -> np.ndarray:
    a = q_normalize(q0)
    b = q_normalize(q1)
    d = float(np.dot(a, b))
    if d < 0:
        b = -b
        d = -d
    if d > 0.9995:
        return q_normalize(a + u * (b - a))
    th0 = math.acos(d)
    th = th0 * u
    s0 = math.sin(th0 - th) / math.sin(th0)
    s1 = math.sin(th) / math.sin(th0)
    return q_normalize(s0 * a + s1 * b)


def euler_zyx_for_display(q: Sequence[float]) -> Tuple[float, float, float]:
    """(yaw, pitch, roll) in radians, intrinsic Z-Y-X.  Display and tests only."""
    w, x, y, z = q_normalize(q)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    sp = 2 * (w * y - z * x)
    sp = max(-1.0, min(1.0, sp))
    pitch = math.asin(sp)
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    return yaw, pitch, roll


def q_from_euler_zyx(yaw: float, pitch: float, roll: float) -> np.ndarray:
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    return np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ])


# --- transforms ------------------------------------------------------------------------------
def identity(target: str, source: str, tag: SourceTag = SourceTag.DEFINITION) -> Transform:
    return Transform(target, source, (1.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0), tag)


def make_transform(target: str, source: str, q_wxyz, t_xyz, tag: SourceTag) -> Transform:
    q = q_normalize(q_wxyz)
    return Transform(target, source, tuple(float(v) for v in q), tuple(float(v) for v in t_xyz), tag)


def rotation(T: Transform) -> np.ndarray:
    return q_to_matrix(T.q_wxyz)


def apply(T: Transform, p: Sequence[float]) -> np.ndarray:
    return rotation(T) @ np.asarray(p, dtype=float) + np.asarray(T.t_xyz)


def rotate_only(T: Transform, v: Sequence[float]) -> np.ndarray:
    return rotation(T) @ np.asarray(v, dtype=float)


def compose(A: Transform, B: Transform) -> Transform:
    """``A @ B``: maps ``B.source`` -> ``A.target``.  Requires ``A.source == B.target``."""
    if A.source != B.target:
        raise FrameMismatchError(
            f"cannot compose T_{A.target}_{A.source} with T_{B.target}_{B.source}: "
            f"'{A.source}' != '{B.target}'")
    q = q_mul(A.q_wxyz, B.q_wxyz)
    t = rotation(A) @ np.asarray(B.t_xyz) + np.asarray(A.t_xyz)
    tag = _weaker_tag(A.source_tag, B.source_tag)
    return make_transform(A.target, B.source, q, t, tag)


def invert(T: Transform) -> Transform:
    qi = q_conj(T.q_wxyz)
    Ri = q_to_matrix(qi)
    ti = -(Ri @ np.asarray(T.t_xyz))
    return make_transform(T.source, T.target, qi, ti, T.source_tag)


_TAG_ORDER = [SourceTag.DEFINITION, SourceTag.CALIBRATED, SourceTag.MEASURED, SourceTag.NOMINAL, SourceTag.UNVALIDATED]


def _weaker_tag(a: SourceTag, b: SourceTag) -> SourceTag:
    return a if _TAG_ORDER.index(a) >= _TAG_ORDER.index(b) else b


def validate_rotation(R: np.ndarray, tol: float = 1e-6) -> None:
    R = np.asarray(R, dtype=float)
    if R.shape != (3, 3):
        raise ValueError("rotation must be 3x3")
    if not np.allclose(R.T @ R, np.eye(3), atol=tol):
        raise ValueError("rotation is not orthonormal")
    if abs(np.linalg.det(R) - 1.0) > tol:
        raise ValueError(f"rotation determinant is {np.linalg.det(R):.6f}, expected +1 (reflection?)")


def is_signed_permutation(R: np.ndarray, tol: float = 1e-6) -> bool:
    A = np.abs(np.asarray(R, dtype=float))
    return bool(np.allclose(A.sum(axis=0), 1, atol=tol) and np.allclose(A.sum(axis=1), 1, atol=tol)
                and np.allclose(A * (1 - A), 0, atol=tol))


def level_from_attitude(q_ref_radar: Sequence[float]) -> np.ndarray:
    """q_level_radar: the attitude with its yaw about the reference +Z removed."""
    yaw, _, _ = euler_zyx_for_display(q_ref_radar)
    q_unyaw = q_from_euler_zyx(-yaw, 0.0, 0.0)
    return q_normalize(q_mul(q_unyaw, q_ref_radar))

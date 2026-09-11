import math

import numpy as np
import pytest
from hypothesis import given, settings
from hypothesis import strategies as st

from goldenfleece import frames as fr
from goldenfleece.types import SourceTag


def _unit_q(draw):
    v = np.array(draw)
    n = np.linalg.norm(v)
    return v / n


@given(st.lists(st.floats(-1, 1, allow_nan=False).filter(lambda x: abs(x) > 1e-3), min_size=4, max_size=4))
@settings(max_examples=200)
def test_quaternion_matrix_roundtrip(q):
    q = fr.q_normalize(q)
    if q[0] < 0:
        q = -q
    R = fr.q_to_matrix(q)
    fr.validate_rotation(R)
    q2 = fr.matrix_to_q(R)
    assert np.allclose(q, q2, atol=1e-9)


def test_compose_invert_is_identity():
    T = fr.make_transform("radar", "imu", fr.q_from_euler_zyx(0.3, -0.2, 1.1), [0.1, -0.2, 0.05], SourceTag.NOMINAL)
    I = fr.compose(T, fr.invert(T))
    assert I.target == "radar" and I.source == "radar"
    assert np.allclose(fr.rotation(I), np.eye(3), atol=1e-12)
    assert np.allclose(I.t_xyz, 0, atol=1e-12)
    p = np.array([1.0, 2.0, 3.0])
    assert np.allclose(fr.apply(fr.invert(T), fr.apply(T, p)), p)


def test_compose_raises_on_frame_mismatch():
    A = fr.identity("radar", "imu")
    B = fr.identity("radar", "imu")
    with pytest.raises(fr.FrameMismatchError):
        fr.compose(A, B)          # A.source ('imu') != B.target ('radar')
    C = fr.identity("imu", "level")
    fr.compose(A, C)              # ok: radar<-imu<-level


def test_compose_order_is_right_to_left():
    T_radar_imu = fr.make_transform("radar", "imu", fr.q_from_euler_zyx(math.pi / 2, 0, 0), [1, 0, 0], SourceTag.NOMINAL)
    T_imu_x = fr.make_transform("imu", "x", [1, 0, 0, 0], [0, 1, 0], SourceTag.NOMINAL)
    T = fr.compose(T_radar_imu, T_imu_x)
    p_x = np.array([0.0, 0.0, 0.0])
    p_radar = fr.apply(T, p_x)
    expected = fr.apply(T_radar_imu, fr.apply(T_imu_x, p_x))
    assert np.allclose(p_radar, expected)
    assert np.allclose(p_radar, [1 - 1, 0, 0])   # (0,1,0) rotated +90deg about z -> (-1,0,0), plus (1,0,0)


def test_yaw_rotates_plus_x_toward_plus_y():
    """Right-handed yaw about +Z takes +X (rearward) toward +Y (rider's RIGHT)."""
    q = fr.q_from_euler_zyx(math.radians(10), 0, 0)
    v = fr.q_rotate(q, [1, 0, 0])
    assert v[1] > 0 and v[0] > 0.98


def test_euler_roundtrip():
    y, p, r = 0.4, -0.3, 0.9
    q = fr.q_from_euler_zyx(y, p, r)
    y2, p2, r2 = fr.euler_zyx_for_display(q)
    assert np.allclose([y, p, r], [y2, p2, r2], atol=1e-12)


def test_level_removes_yaw_keeps_roll_pitch():
    q = fr.q_from_euler_zyx(1.2, 0.1, -0.3)
    ql = fr.level_from_attitude(q)
    y, p, r = fr.euler_zyx_for_display(ql)
    assert abs(y) < 1e-9
    assert np.allclose([p, r], [0.1, -0.3], atol=1e-9)


def test_signed_permutation_and_reflection_detection():
    assert fr.is_signed_permutation(np.diag([1, -1, -1]))
    assert not fr.is_signed_permutation(fr.q_to_matrix(fr.q_from_euler_zyx(0.1, 0, 0)))
    with pytest.raises(ValueError):
        fr.validate_rotation(np.diag([1, 1, -1]))   # det -1 reflection


def test_slerp_endpoints():
    a = fr.q_from_euler_zyx(0, 0, 0)
    b = fr.q_from_euler_zyx(0.5, 0, 0)
    assert np.allclose(fr.q_slerp(a, b, 0), a)
    assert np.allclose(np.abs(fr.q_slerp(a, b, 1)), np.abs(b))
    m = fr.q_slerp(a, b, 0.5)
    assert abs(fr.euler_zyx_for_display(m)[0] - 0.25) < 1e-9

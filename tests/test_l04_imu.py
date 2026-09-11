import math

import numpy as np
import pytest

from goldenfleece import frames as fr
from goldenfleece.l04_imu_decode.eskf import G, Eskf, EskfParams
from goldenfleece.l04_imu_decode.imu_decoder import ImuDecoder
from goldenfleece.types import ImuHealth, ImuKind, RawImuSample


def _feed(dec, t0, t1, gyro, accel, fg=200.0, fa=100.0, seed=0):
    rng = np.random.default_rng(seed)
    tg = np.arange(t0, t1, 1 / fg)
    ta = np.arange(t0 + 0.001, t1, 1 / fa)
    samples = []
    seq = 0
    for t in tg:
        seq += 1
        samples.append(RawImuSample(t, ImuKind.GYRO, tuple(np.asarray(gyro(t)) + rng.normal(0, 0.002, 3)), seq))
    for t in ta:
        seq += 1
        samples.append(RawImuSample(t, ImuKind.ACCEL, tuple(np.asarray(accel(t)) + rng.normal(0, 0.03, 3)), seq))
    samples.sort(key=lambda s: s.t)
    for s in samples:
        dec.ingest(s)


def test_static_reads_plus_g_on_up_axis(cfg):
    """+9.81 on +Z at rest and level (specific force, not acceleration)."""
    dec = ImuDecoder(cfg.pipeline.imu, cfg.frames.T_radar_imu)
    _feed(dec, 0.0, 2.0, lambda t: [0, 0, 0], lambda t: [0, 0, G])
    st = dec.state_at(1.9)
    assert st.health is ImuHealth.OK
    assert abs(st.roll) < 0.01 and abs(st.pitch) < 0.01
    assert not st.in_motion
    assert dec.n_accel_applied > 100


def test_static_tilt_converges_to_roll(cfg):
    dec = ImuDecoder(cfg.pipeline.imu, cfg.frames.T_radar_imu)
    roll = math.radians(20)
    q = fr.q_from_euler_zyx(0, 0, roll)                    # body rolled 20 deg about +X
    f_body = fr.q_to_matrix(q).T @ np.array([0, 0, G])
    _feed(dec, 0.0, 3.0, lambda t: [0, 0, 0], lambda t: f_body)
    st = dec.state_at(2.9)
    assert abs(st.roll - roll) < 0.02
    ql = np.array(st.q_level_radar)
    up_in_level = fr.q_to_matrix(ql) @ (fr.q_to_matrix(q).T @ np.array([0, 0, 1]))
    assert np.allclose(up_in_level, [0, 0, 1], atol=0.03)   # T_level_radar levels the body


def test_constant_yaw_rate_integrates_and_yaw_sigma_grows(cfg):
    dec = ImuDecoder(cfg.pipeline.imu, cfg.frames.T_radar_imu)
    rate = 0.5
    _feed(dec, 0.0, 4.0, lambda t: [0, 0, rate], lambda t: [0, 0, G])
    d = dec.delta_yaw(2.5, 3.5)            # inside the 2 s buffer
    assert abs(d - 1.0 * rate) < 0.03
    s1 = dec.state_at(2.5)
    s3 = dec.state_at(3.5)
    assert s3.yaw_sigma > s1.yaw_sigma > 0       # unobservable: covariance grows


def test_gyro_bias_is_estimated(cfg):
    dec = ImuDecoder(cfg.pipeline.imu, cfg.frames.T_radar_imu)
    bias = np.array([0.02, -0.01, 0.0])
    _feed(dec, 0.0, 20.0, lambda t: bias, lambda t: [0, 0, G])
    st = dec.state_at(19.9)
    assert abs(st.gyro_bias[0] - 0.02) < 0.006 and abs(st.gyro_bias[1] + 0.01) < 0.006
    assert abs(st.roll) < 0.02 and abs(st.pitch) < 0.02


def test_accel_gated_in_coordinated_turn(cfg):
    """In a steady lean the resultant specific force points along body-up; the norm exceeds g and the
    gyro shows rotation, so the accelerometer update must be gated and roll must follow the gyro."""
    dec = ImuDecoder(cfg.pipeline.imu, cfg.frames.T_radar_imu)
    _feed(dec, 0.0, 2.0, lambda t: [0, 0, 0], lambda t: [0, 0, G])       # settle level
    n0 = dec.n_accel_applied
    lean_rate = math.radians(20)                                            # roll to 20 deg over 1 s
    _feed(dec, 2.0, 3.0, lambda t: [lean_rate, 0, 0.6], lambda t: [0, 0, 11.0], seed=1)   # norm 11 > g + 0.5
    assert dec.n_accel_applied == n0                                        # everything gated
    st = dec.state_at(2.99)
    assert abs(st.roll - math.radians(20)) < 0.03


def test_no_data_and_stale_flags(cfg):
    dec = ImuDecoder(cfg.pipeline.imu, cfg.frames.T_radar_imu)
    st = dec.state_at(5.0)
    assert st.health is ImuHealth.NO_DATA and st.q_level_radar == (1, 0, 0, 0)
    _feed(dec, 0.0, 1.0, lambda t: [0, 0, 0], lambda t: [0, 0, G])
    assert dec.state_at(0.5).health is ImuHealth.OK
    assert dec.state_at(1.5).health is ImuHealth.STALE
    assert dec.state_at(0.99, t_now=2.0).health is ImuHealth.NO_DATA


def test_extrinsic_axis_map_applied(cfg, tmp_path):
    """IMU mounted with its X along radar -Y (90 deg about Z): a body roll about radar X shows up as a
    rotation about the IMU's -Y... the decoder must map it back so the level frame is right."""
    T = fr.make_transform("radar", "imu", fr.q_from_euler_zyx(math.pi / 2, 0, 0), [0, 0, 0], cfg.frames.T_radar_imu.source_tag)
    dec = ImuDecoder(cfg.pipeline.imu, T)
    R_imu_radar = fr.q_to_matrix(T.q_wxyz).T
    q_body = fr.q_from_euler_zyx(0, 0, math.radians(15))              # radar rolled 15 deg
    f_radar = fr.q_to_matrix(q_body).T @ np.array([0, 0, G])
    f_imu = R_imu_radar @ f_radar
    _feed(dec, 0.0, 3.0, lambda t: [0, 0, 0], lambda t: f_imu)
    st = dec.state_at(2.9)
    assert abs(st.roll - math.radians(15)) < 0.02 and abs(st.pitch) < 0.02


def test_motion_indicator(cfg):
    dec = ImuDecoder(cfg.pipeline.imu, cfg.frames.T_radar_imu)
    rng = np.random.default_rng(5)
    _feed(dec, 0.0, 2.0, lambda t: rng.normal(0, 0.1, 3), lambda t: [0, 0, G] + rng.normal(0, 0.6, 3))
    assert dec.state_at(1.9).in_motion


def test_eskf_direct_init():
    e = Eskf(EskfParams())
    e.init_from_gravity(np.array([0, 0, G]))
    assert np.allclose(e.q, [1, 0, 0, 0])
    e2 = Eskf(EskfParams())
    q = fr.q_from_euler_zyx(0, 0.3, -0.2)
    e2.init_from_gravity(fr.q_to_matrix(q).T @ np.array([0, 0, G]))
    _, p, r = fr.euler_zyx_for_display(e2.q)
    assert abs(p - 0.3) < 1e-6 and abs(r + 0.2) < 1e-6

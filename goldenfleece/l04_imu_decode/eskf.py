"""Error-state Kalman filter for attitude + gyro bias.

Nominal state: q_ref_body ([w,x,y,z]; v_ref = R(q) v_body) and gyro bias b.
Error state: [dtheta (body-frame local perturbation), db], 6x6 covariance.
Reference frame: gravity-aligned (+Z up), yaw arbitrary and unobservable.
Accelerometer update = gravity direction only, gated (coordinated-turn degeneracy).
Pure: dt and samples are passed in; nothing here reads a clock.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

from .. import frames as fr

G = 9.80665


@dataclass
class EskfParams:
    gyro_noise_rad_s: float = 0.004
    gyro_bias_rw_rad_s2: float = 0.0002
    accel_dir_noise_rad: float = 0.08
    accel_norm_gate_mps2: float = 0.5
    gyro_rate_gate_rad_s: float = 0.35
    init_sigma_att_rad: float = 0.3
    init_sigma_bias_rad_s: float = 0.02


def _skew(v: np.ndarray) -> np.ndarray:
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]], dtype=float)


class Eskf:
    def __init__(self, p: EskfParams) -> None:
        self.p = p
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.b = np.zeros(3)
        self.P = np.diag([p.init_sigma_att_rad ** 2] * 3 + [p.init_sigma_bias_rad_s ** 2] * 3)
        self.initialised = False
        self.n_prop = 0
        self.n_upd = 0
        self.n_gated_norm = 0
        self.n_gated_rate = 0
        self._last_omega = np.zeros(3)

    # -- initialisation from a (near) static accelerometer sample --------------------------------
    def init_from_gravity(self, f_body: np.ndarray) -> None:
        z_b = f_body / np.linalg.norm(f_body)          # body-frame up direction
        e_z = np.array([0.0, 0.0, 1.0])
        # rotation taking body-up to ref-up with zero yaw: axis = z_b x e_z
        axis = np.cross(z_b, e_z)
        s = np.linalg.norm(axis)
        c = float(np.dot(z_b, e_z))
        if s < 1e-9:
            q = np.array([1.0, 0.0, 0.0, 0.0]) if c > 0 else np.array([0.0, 1.0, 0.0, 0.0])
        else:
            ang = math.atan2(s, c)
            q = fr.q_from_rotvec(axis / s * ang)
        self.q = fr.q_normalize(q)
        self.initialised = True

    # -- propagation -----------------------------------------------------------------------------
    def propagate(self, omega_meas: np.ndarray, dt: float) -> None:
        if dt <= 0 or dt > 0.5:
            return
        w = omega_meas - self.b
        self._last_omega = w
        dq = fr.q_from_rotvec(w * dt)
        self.q = fr.q_normalize(fr.q_mul(self.q, dq))
        Rw = fr.q_to_matrix(dq)
        F = np.eye(6)
        F[0:3, 0:3] = Rw.T
        F[0:3, 3:6] = -np.eye(3) * dt
        Q = np.zeros((6, 6))
        Q[0:3, 0:3] = np.eye(3) * (self.p.gyro_noise_rad_s ** 2) * dt
        Q[3:6, 3:6] = np.eye(3) * (self.p.gyro_bias_rw_rad_s2 ** 2) * dt
        self.P = F @ self.P @ F.T + Q
        self.n_prop += 1

    # -- gravity-direction update ----------------------------------------------------------------
    def update_gravity(self, f_body: np.ndarray, omega_for_gate: Optional[np.ndarray] = None) -> bool:
        n = float(np.linalg.norm(f_body))
        if n < 1e-6:
            return False
        if abs(n - G) > self.p.accel_norm_gate_mps2:
            self.n_gated_norm += 1
            return False
        w = self._last_omega if omega_for_gate is None else omega_for_gate
        if float(np.linalg.norm(w)) > self.p.gyro_rate_gate_rad_s:
            self.n_gated_rate += 1
            return False
        if not self.initialised:
            self.init_from_gravity(f_body)
            return True
        R = fr.q_to_matrix(self.q)
        e_z = np.array([0.0, 0.0, 1.0])
        h = R.T @ e_z                       # predicted body-frame up direction
        z = f_body / n
        y = z - h
        H = np.zeros((3, 6))
        H[:, 0:3] = _skew(h)
        Rm = np.eye(3) * (self.p.accel_dir_noise_rad ** 2)
        S = H @ self.P @ H.T + Rm
        K = self.P @ H.T @ np.linalg.inv(S)
        dx = K @ y
        self.q = fr.q_normalize(fr.q_mul(self.q, fr.q_from_rotvec(dx[0:3])))
        self.b = self.b + dx[3:6]
        I_KH = np.eye(6) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ Rm @ K.T
        self.n_upd += 1
        return True

    # -- accessors --------------------------------------------------------------------------------
    def yaw_sigma(self) -> float:
        R = fr.q_to_matrix(self.q)
        e_z = np.array([0.0, 0.0, 1.0])
        v = R.T @ e_z                        # ref +Z expressed in body: yaw error is rotation about it
        return float(math.sqrt(max(0.0, v @ self.P[0:3, 0:3] @ v)))

    def roll_pitch(self):
        _, pitch, roll = fr.euler_zyx_for_display(self.q)
        return roll, pitch

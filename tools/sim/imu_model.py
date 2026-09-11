"""BNO085 model from the same kinematic truth.

Gyro: angular velocity of the radar frame expressed in the IMU frame + bias + noise.
Accelerometer: specific force f = R_imu_w (a_w - g_w): at rest and level it reads
+9.81 m/s^2 on the up axis.  Includes centripetal acceleration in turns.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

from goldenfleece.types import ImuKind, RawImuSample
from .kinematics import G, RiderKinematics


@dataclass
class ImuModelParams:
    gyro_rate_hz: float = 200.0
    accel_rate_hz: float = 100.0
    gyro_noise_rad_s: float = 0.003
    gyro_bias_rad_s: Tuple[float, float, float] = (0.010, -0.005, 0.020)
    accel_noise_mps2: float = 0.05
    accel_bias_mps2: Tuple[float, float, float] = (0.02, -0.01, 0.03)
    timestamp_jitter_s: float = 0.0005
    R_imu_radar: Optional[np.ndarray] = None       # signed permutation; identity if None
    fault_t_start: Optional[float] = None          # no samples after this
    fault_t_end: Optional[float] = None
    seed: int = 1


class ImuModel:
    def __init__(self, rider: RiderKinematics, params: ImuModelParams) -> None:
        self.rider = rider
        self.p = params
        self.rng = np.random.default_rng(params.seed)
        self.R = np.eye(3) if params.R_imu_radar is None else np.asarray(params.R_imu_radar, dtype=float)
        self.seq = 0
        self._next_g = 0.0
        self._next_a = 0.0

    def _faulted(self, t: float) -> bool:
        p = self.p
        return p.fault_t_start is not None and p.fault_t_start <= t <= (p.fault_t_end if p.fault_t_end is not None else 1e18)

    def samples_until(self, t_end: float) -> List[RawImuSample]:
        out: List[RawImuSample] = []
        dtg = 1.0 / self.p.gyro_rate_hz
        dta = 1.0 / self.p.accel_rate_hz
        while self._next_g <= t_end or self._next_a <= t_end:
            if self._next_g <= self._next_a:
                t = self._next_g
                self._next_g += dtg
                if not self._faulted(t):
                    w_r = self.rider.body_rates(t)
                    w = self.R @ w_r + np.array(self.p.gyro_bias_rad_s) + self.rng.normal(0, self.p.gyro_noise_rad_s, 3)
                    self.seq += 1
                    out.append(RawImuSample(t=t + float(self.rng.normal(0, self.p.timestamp_jitter_s)), kind=ImuKind.GYRO,
                                            values=tuple(float(v) for v in w), seq=self.seq))
            else:
                t = self._next_a
                self._next_a += dta
                if not self._faulted(t):
                    st = self.rider.state(t)
                    g_w = np.array([0.0, 0.0, -G])
                    f_r = st.R_r_w @ (st.a_w - g_w)
                    if st.speed > 0.3:
                        f_r = f_r + self.rng.normal(0, self.rider.profile.vibration_rms_mps2, 3)
                    f = self.R @ f_r + np.array(self.p.accel_bias_mps2) + self.rng.normal(0, self.p.accel_noise_mps2, 3)
                    self.seq += 1
                    out.append(RawImuSample(t=t + float(self.rng.normal(0, self.p.timestamp_jitter_s)), kind=ImuKind.ACCEL,
                                            values=tuple(float(v) for v in f), seq=self.seq))
        out.sort(key=lambda s: s.t)
        return out

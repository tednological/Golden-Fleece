"""Simulator-driven RadarFrameSource / ImuSource for running the real Runner deterministically.

The radar source advances the FakeClock: each ``get`` produces the next frame (or, during a gap
or silence, advances by the timeout and returns None so the runner ticks).  IMU samples up to
the current clock time are pushed into the IMU source before the frame is returned.
"""
from __future__ import annotations

from typing import List, Optional

from goldenfleece.clock import FakeClock
from goldenfleece.l02_imu_data_input.source import QueueImuSource
from goldenfleece.types import HealthEvent, RawRadarFrame
from .imu_model import ImuModel
from .radar_model import FrameTruth, RadarModel


class SimRadarSource:
    def __init__(self, radar: RadarModel, imu: ImuModel, imu_source: QueueImuSource, clock: FakeClock, duration_s: float,
                 compute_delay_s: float = 0.0) -> None:
        self.radar = radar
        self.imu = imu
        self.imu_source = imu_source
        self.clock = clock
        self.duration_s = duration_s
        self.compute_delay_s = compute_delay_s
        self.k = 0
        self.truths: List[FrameTruth] = []
        self.last_truth: Optional[FrameTruth] = None
        self.finished = False
        self._events: list = []
        self._pending: Optional[tuple] = None

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def push_event(self, ev: HealthEvent) -> None:
        self._events.append(ev)

    def drain_events(self) -> list:
        ev, self._events = self._events, []
        return ev

    def get(self, timeout_s: float) -> Optional[RawRadarFrame]:
        T = self.radar.T
        while True:
            t_mid = 0.5 * T + self.k * T
            if t_mid > self.duration_s:
                self.finished = True
                self.clock.advance(timeout_s)
                self._push_imu(self.clock.now())
                return None
            t_header = t_mid + T / 2 + self.radar.p.sensor_delay_s
            if t_header > self.clock.now() + timeout_s:
                # next frame is beyond this wait: tick
                self.clock.advance(timeout_s)
                self._push_imu(self.clock.now())
                return None
            raw, truth = self.radar.frame(t_mid)
            self.k += 1
            self.truths.append(truth)
            self.last_truth = truth
            if raw is None:
                continue
            self.clock.set(max(self.clock.now(), raw.t_header))
            self._push_imu(self.clock.now())
            return raw

    def _push_imu(self, t: float) -> None:
        for s in self.imu.samples_until(t):
            self.imu_source.push(s)

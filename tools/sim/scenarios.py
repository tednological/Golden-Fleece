"""The §13.2 scenario suite.  One kinematic truth drives both sensors."""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional

import numpy as np

from .imu_model import ImuModelParams
from .kinematics import RiderProfile, ShoulderCheck
from .radar_model import RadarModelParams
from .road import Road, RoadSegment
from .world import BIKE_POINTS, Scatterer, Vehicle, World


@dataclass
class Scenario:
    name: str
    duration_s: float
    world: World
    rider: RiderProfile
    radar: RadarModelParams = field(default_factory=RadarModelParams)
    imu: ImuModelParams = field(default_factory=ImuModelParams)
    notes: str = ""
    expect_warning: bool = True          # at least one vehicle should trigger WARNING+
    fault_onsets: Dict[str, float] = field(default_factory=dict)     # health bit name -> truth onset time


def _clutter(seed: int, density: float, s_min=-60.0, s_max=400.0) -> List[Scatterer]:
    return World.roadside_clutter(np.random.default_rng(seed), s_min, s_max, density)


def _rider(v=6.0, yaw_amp_deg=8.0, **kw) -> RiderProfile:
    return RiderProfile(speed_keypoints=((0.0, v),), torso_yaw_amp_rad=math.radians(yaw_amp_deg), torso_yaw_period_s=3.5, **kw)


def overtake(closing: float, lateral: float, name: Optional[str] = None, v_rider=6.0, duration=None, clutter_density=0.15, seed=1) -> Scenario:
    d0 = -45.0
    dur = duration or (abs(d0) / closing + 4.0)
    world = World(Road(), vehicles=[Vehicle("car", s0=d0, speed_mps=v_rider + closing, lateral_m=lateral)], scatterers=_clutter(seed, clutter_density))
    side = "left" if lateral > 0 else "right"
    return Scenario(name or f"overtake_{side}_{closing:g}mps", dur, world, _rider(v_rider), radar=RadarModelParams(seed=seed),
                    notes=f"single overtake, closing {closing} m/s, {abs(lateral)} m to the rider's {side}")


def close_pass() -> Scenario:
    world = World(Road(), vehicles=[Vehicle("car", s0=-40.0, speed_mps=6.0 + 8.0, lateral_m=-1.0)], scatterers=_clutter(2, 0.15))
    return Scenario("close_pass_1m", 9.0, world, _rider(6.0), radar=RadarModelParams(seed=2), notes="1 m lateral, exercises FOV_EXIT")


def pacer() -> Scenario:
    # same-speed pacer drifting in and out of the blind band, then finally overtaking
    kp = [(0.0, 6.3), (4.0, 5.8), (8.0, 6.4), (12.0, 5.9), (16.0, 6.6), (20.0, 8.5), (30.0, 8.5)]
    world = World(Road(), vehicles=[Vehicle("pacer", s0=-12.0, speed_mps=6.0, lateral_m=2.5, speed_keypoints=kp)], scatterers=_clutter(3, 0.15))
    return Scenario("same_speed_pacer", 26.0, world, _rider(6.0), radar=RadarModelParams(seed=3),
                    notes="relative speed oscillates through the Doppler blind band; final overtake must warn")


def two_vehicles() -> Scenario:
    world = World(Road(), vehicles=[Vehicle("fast", s0=-60.0, speed_mps=6.0 + 14.0, lateral_m=3.5),
                                    Vehicle("slow", s0=-25.0, speed_mps=6.0 + 4.0, lateral_m=-2.5)], scatterers=_clutter(4, 0.15))
    return Scenario("two_vehicles_crossing", 9.0, world, _rider(6.0), radar=RadarModelParams(seed=4),
                    notes="two ranges, azimuths cross as the fast car passes the slow one")


def shoulder_check() -> Scenario:
    prof = _rider(6.0, shoulder_checks=[ShoulderCheck(2.0, 1.2, math.radians(50)), ShoulderCheck(4.5, 1.0, math.radians(-40))])
    world = World(Road(), vehicles=[Vehicle("car", s0=-50.0, speed_mps=6.0 + 10.0, lateral_m=3.0)], scatterers=_clutter(5, 0.15))
    return Scenario("overtake_during_shoulder_check", 9.0, world, prof, radar=RadarModelParams(seed=5),
                    notes="50 deg and -40 deg shoulder checks while a car closes at 10 m/s")


def cornering() -> Scenario:
    v = 8.0
    kappa = math.tan(math.radians(20)) * 9.80665 / (v * v)           # 20 deg lean
    road = Road([RoadSegment(40.0, 0.0), RoadSegment(25.0, kappa), RoadSegment(1000.0, 0.0)])     # ~80 deg corner, 18 m radius
    world = World(road, vehicles=[Vehicle("car", s0=-40.0, speed_mps=v + 10.0, lateral_m=3.0)], scatterers=_clutter(6, 0.15, -60, 600))
    return Scenario("cornering_lean_20deg_overtake", 12.0, world, RiderProfile(speed_keypoints=((0.0, v),), torso_yaw_amp_rad=math.radians(5)),
                    radar=RadarModelParams(seed=6), notes="20 deg coordinated lean during an overtake")


def open_road() -> Scenario:
    world = World(Road(), vehicles=[Vehicle("car", s0=(6.0 * 8.0 - 60.0) - 18.0 * 8.0, speed_mps=6.0 + 12.0, lateral_m=3.0, t_appear=8.0)], scatterers=[])
    return Scenario("open_road_no_clutter", 20.0, world, _rider(6.0), radar=RadarModelParams(seed=7, false_alarm_rate_per_frame=0.02),
                    notes="no clutter: ego-motion invalid; overtake at t=8 s; also measures blockage false alarms")


def dense_clutter() -> Scenario:
    world = World(Road(), vehicles=[Vehicle("car", s0=-50.0, speed_mps=9.0 + 12.0, lateral_m=3.0)], scatterers=_clutter(8, 2.5))
    return Scenario("dense_clutter_cap", 8.0, world, _rider(9.0), radar=RadarModelParams(seed=8),
                    notes="2.5 scatterers/m at 9 m/s: the 12-target cap fills")


def stopped_at_light() -> Scenario:
    world = World(Road(), vehicles=[Vehicle("car", s0=-40.0, speed_mps=8.0, lateral_m=2.5)], scatterers=_clutter(9, 0.3))
    return Scenario("stopped_at_light", 7.0, world, RiderProfile(speed_keypoints=((0.0, 0.0),), vibration_rms_mps2=0.0),
                    radar=RadarModelParams(seed=9), notes="rider stopped; car approaching at 8 m/s; clutter invisible")


def radar_blocked() -> Scenario:
    world = World(Road(), vehicles=[Vehicle("car", s0=(6.0 * 12.0 - 60.0) - 18.0 * 12.0, speed_mps=6.0 + 12.0, lateral_m=3.0, t_appear=12.0)], scatterers=_clutter(10, 0.4))
    sc = Scenario("radar_blocked_mid_ride", 20.0, world, _rider(6.0), radar=RadarModelParams(seed=10, blockage_t_start=5.0),
                  notes="radome covered at t=5 s: valid empty frames; car at t=12 s is missed by design (announce, don't warn)",
                  expect_warning=False)
    sc.fault_onsets["RADAR_POSSIBLY_BLOCKED"] = 5.0
    return sc


def radar_gaps() -> Scenario:
    sc = overtake(10.0, 3.0, name="radar_frame_gaps", seed=11)
    sc.radar.gap_probability = 0.3
    sc.fault_onsets["RADAR_GAPS"] = 0.0
    return sc


def radar_silence() -> Scenario:
    sc = overtake(10.0, 3.0, name="radar_silence", seed=12, duration=10.0)
    sc.radar.silence_t_start = 3.0
    sc.radar.silence_t_end = 6.0
    sc.fault_onsets["RADAR_SILENT"] = 3.0
    return sc


def imu_fault() -> Scenario:
    sc = overtake(10.0, 3.0, name="imu_fault", seed=13)
    sc.imu.fault_t_start = 2.0
    sc.fault_onsets["IMU_FAULT"] = 2.0
    return sc


def all_scenarios() -> List[Scenario]:
    return [
        overtake(5.0, 3.0), overtake(10.0, 3.0), overtake(20.0, 3.0), overtake(5.0, -3.0), overtake(10.0, -3.0), overtake(20.0, -3.0),
        close_pass(), pacer(), two_vehicles(), shoulder_check(), cornering(), open_road(), dense_clutter(), stopped_at_light(),
        radar_blocked(), radar_gaps(), radar_silence(), imu_fault(),
    ]


BY_NAME: Dict[str, Callable[[], Scenario]] = {s.name: (lambda s=s: s) for s in all_scenarios()}

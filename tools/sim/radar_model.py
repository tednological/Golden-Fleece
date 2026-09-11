"""K-LD7 measurement model driven by the shared kinematic truth.

Datasheet-derived behaviour that the model reproduces:
  * raw targets are Doppler-FFT bins above a threshold; scatterers sharing a bin
    MERGE into one target whose distance and angle come from the composite phase;
  * one distance per target from FSK phase (unambiguous range = RRAI max range);
  * angle from Rx1/Rx2 phase, lambda/2 spacing, 1 deg quantisation, +/-90 deg;
  * speed sign: positive = receding; angle sign: positive = sensor-right = rider's LEFT;
  * PDAT holds at most 12 targets (cap policy is configurable: unverified);
  * targets above the speed range alias; targets beyond the range setting wrap.
Also modelled: beam pattern (80/34 deg, sidelobes), blind band, angle noise,
false alarms, frame gaps, blockage.  Output is wire-native (RawRadarFrame).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

from goldenfleece.types import RawRadarFrame, RawRadarTarget
from .kinematics import RiderKinematics, RiderState
from .world import World

SPEED_MAX_KMH = {0: 12.5, 1: 25.0, 2: 50.0, 3: 100.0}
RANGE_MAX_M = {0: 5.0, 1: 10.0, 2: 30.0, 3: 100.0}
RANGE_RES_M = {0: 0.05, 1: 0.10, 2: 0.30, 3: 1.00}
FRAME_S = {0: 0.229, 1: 0.114, 2: 0.057, 3: 0.029}
N_BINS = 256


@dataclass
class RadarModelParams:
    rspi: int = 3
    rrai: int = 2
    thof_db: float = 30.0
    dedi: int = 2                          # 0 receding only, 1 approaching only, 2 both
    misp_pct: int = 0                      # % of speed range
    sensor_delay_s: float = 0.010
    usb_jitter_s: float = 0.002
    blind_band_mps: float = 0.5
    angle_noise_deg: float = 1.5
    range_noise_m: float = 0.10
    false_alarm_rate_per_frame: float = 0.05
    gap_probability: float = 0.0
    cap_policy: str = "weakest_first"      # weakest_first | fft_order | random   (UNVERIFIED which the sensor does)
    blockage_t_start: Optional[float] = None
    blockage_t_end: Optional[float] = None
    blockage_atten_db: float = 40.0
    silence_t_start: Optional[float] = None   # sensor stops answering (RADAR_SILENT)
    silence_t_end: Optional[float] = None
    range_wrap: bool = True
    calib_db: float = 80.0     # cars (10 m^2) detectable to ~32 m, persons (1 m^2) to ~18 m: datasheet typical 30 / 15 m
    sidelobe_db: float = -15.0
    seed: int = 0
    max_targets: int = 12


@dataclass
class VehicleTruth:
    name: str
    r: float
    az: float                   # radar frame, rad (+ = rider's RIGHT)
    elev: float
    v_radial: float             # + receding
    in_beam: bool
    t_arrival: Optional[float]  # time until the vehicle reaches the rider's longitudinal position (None if not closing)
    lateral_m: float            # + = rider's LEFT (sim_world convention)
    side: str                   # LEFT | CENTER | RIGHT
    s_rel: float                # vehicle arc length minus rider arc length (negative = behind)
    closing_speed: float


@dataclass
class FrameTruth:
    t_mid: float
    frame_number: int
    vehicles: List[VehicleTruth]
    rider: RiderState
    n_scatter_visible: int
    n_above_threshold: int
    n_bins_after_merge: int
    n_clutter_bins: int
    cap_hit: bool
    gap: bool
    blocked: bool
    silent: bool


def beam_gain_db(az: float, el: float, sidelobe_db: float) -> float:
    """One-way pattern (dB).  -3 dB at +/-40 deg azimuth and +/-17 deg elevation."""
    a = abs(az) / math.radians(40.0)
    e = abs(el) / math.radians(17.0)
    main = -3.0 * (a * a + e * e)
    if abs(az) > math.radians(52.0) or abs(el) > math.radians(28.0):
        return max(sidelobe_db - 3.0 * (e * e), -40.0) + 3.0 * math.cos(6 * az) - 3.0
    return main


class RadarModel:
    def __init__(self, world: World, rider: RiderKinematics, params: RadarModelParams) -> None:
        self.world = world
        self.rider = rider
        self.p = params
        self.rng = np.random.default_rng(params.seed)
        self.T = FRAME_S[params.rspi]
        self.v_max_kmh = SPEED_MAX_KMH[params.rspi]
        self.bin_kmh = 2 * self.v_max_kmh / N_BINS
        self.r_max = RANGE_MAX_M[params.rrai]
        self.r_res = RANGE_RES_M[params.rrai]
        self.frame_counter = 0
        self.seq = 0
        self._last_emitted_fn = None

    # -- helpers ---------------------------------------------------------------------------
    def _vehicle_points(self, t: float, st: RiderState):
        """Yield (p_w, v_w, rcs, tag) for every vehicle scatter point."""
        road = self.world.road
        out = []
        for veh in self.world.vehicles:
            if not veh.active(t):
                continue
            s_v = veh.arc_length(t)
            vv = veh.speed(t)
            x, y, th = road.pose(s_v)
            f = np.array([math.cos(th), math.sin(th), 0.0])
            l = np.array([-math.sin(th), math.cos(th), 0.0])
            base = road.world_point(s_v, veh.lateral_m, 0.0)
            for pt in veh.points:
                p_w = base + pt.dx * f + pt.dy * l + np.array([0, 0, pt.dz])
                v_w = vv * pt.velocity_scale * f
                out.append((p_w, v_w, pt.rcs_m2, veh.name))
        return out

    def _static_points(self):
        road = self.world.road
        return [(road.world_point(sc.s, sc.lateral_m, sc.z), np.zeros(3), sc.rcs_m2, "clutter") for sc in self.world.scatterers]

    def _observe(self, p_w, v_w, st: RiderState):
        rel = p_w - st.p_w
        p_r = st.R_r_w @ rel
        r = float(np.linalg.norm(p_r))
        if r < 0.3:
            return None
        az = math.atan2(p_r[1], p_r[0])
        el = math.asin(max(-1.0, min(1.0, p_r[2] / r)))
        v_rad = float(rel @ (v_w - st.v_w) / r)
        return r, az, el, v_rad

    def _vehicle_truth(self, t: float, st: RiderState) -> List[VehicleTruth]:
        out = []
        for veh in self.world.vehicles:
            if not veh.active(t):
                continue
            s_v = veh.arc_length(t)
            p_w = self.world.road.world_point(s_v, veh.lateral_m, 0.5)
            th = self.world.road.heading(s_v)
            v_w = veh.speed(t) * np.array([math.cos(th), math.sin(th), 0.0])
            obs = self._observe(p_w, v_w, st)
            if obs is None:
                continue
            r, az, el, v_rad = obs
            s_rel = s_v - st.s
            closing = veh.speed(t) - st.speed
            t_arr = (-s_rel / closing) if (closing > 0.05 and s_rel < 0) else None
            side = "LEFT" if veh.lateral_m > 0.5 else ("RIGHT" if veh.lateral_m < -0.5 else "CENTER")
            out.append(VehicleTruth(veh.name, r, az, el, v_rad, abs(az) <= math.radians(40) and abs(el) <= math.radians(17),
                                    t_arr, veh.lateral_m, side, s_rel, closing))
        return out

    # -- frame generation ------------------------------------------------------------------------
    def frame(self, t_mid: float) -> Tuple[Optional[RawRadarFrame], FrameTruth]:
        p = self.p
        st = self.rider.state(t_mid)
        fn = self.frame_counter
        self.frame_counter += 1
        truth_veh = self._vehicle_truth(t_mid, st)
        silent = p.silence_t_start is not None and p.silence_t_start <= t_mid <= (p.silence_t_end or 1e18)
        blocked = p.blockage_t_start is not None and p.blockage_t_start <= t_mid <= (p.blockage_t_end or 1e18)
        gap = self.rng.random() < p.gap_probability

        points = self._vehicle_points(t_mid, st) + self._static_points()
        cands = []           # (mag_db, r, az, el, v_rad, tag)
        n_visible = 0
        thresh = 30.0 + (p.thof_db - 30.0)
        atten = p.blockage_atten_db if blocked else 0.0
        for p_w, v_w, rcs, tag in points:
            obs = self._observe(p_w, v_w, st)
            if obs is None:
                continue
            r, az, el, v_rad = obs
            if abs(az) > math.radians(89):
                continue
            n_visible += 1
            g = beam_gain_db(az, el, p.sidelobe_db)
            mag = 10 * math.log10(rcs) - 40 * math.log10(max(r, 0.5)) + 2 * g + p.calib_db - atten
            mag += self.rng.normal(0, 1.0)
            if mag < thresh:
                continue
            if abs(v_rad) < p.blind_band_mps:
                continue
            cands.append((mag, r, az, el, v_rad, tag))
        n_above = len(cands)

        # Doppler binning (aliasing beyond +/- v_max) and merging within a bin
        bins: Dict[int, List[Tuple[float, float, float, float, str]]] = {}
        for mag, r, az, el, v_rad, tag in cands:
            v_kmh = v_rad * 3.6
            b = int(round(v_kmh / self.bin_kmh))
            # alias into [-128, 127]
            b = ((b + N_BINS // 2) % N_BINS) - N_BINS // 2
            bins.setdefault(b, []).append((mag, r, az, v_rad, tag))
        targets: List[Tuple[float, int, float, float, str]] = []   # (mag, bin, r_rep, az_rep, tag)
        n_clutter_bins = 0
        for b, items in bins.items():
            amps = np.array([10 ** (m / 20) for m, *_ in items])
            azs = np.array([it[2] for it in items])
            rs = np.array([it[1] for it in items])
            A = np.sum(amps * np.exp(1j * math.pi * np.sin(azs)))
            phase = math.atan2(A.imag, A.real)
            az_rep = math.asin(max(-1.0, min(1.0, phase / math.pi)))
            B = np.sum(amps * np.exp(1j * 2 * math.pi * (rs / self.r_max)))
            phr = math.atan2(B.imag, B.real) % (2 * math.pi)
            r_rep = phr / (2 * math.pi) * self.r_max
            if not p.range_wrap:
                r_rep = float(np.average(rs, weights=amps))
            mag_rep = 10 * math.log10(float(np.sum(amps ** 2)))
            tag = max(items, key=lambda it: it[0])[4]
            if tag == "clutter":
                n_clutter_bins += 1
            targets.append((mag_rep, b, r_rep, az_rep, tag))
        n_after_merge = len(targets)

        # false alarms
        n_fa = self.rng.poisson(p.false_alarm_rate_per_frame)
        for _ in range(n_fa):
            b = int(self.rng.integers(-N_BINS // 2, N_BINS // 2))
            if b == 0:
                continue
            targets.append((thresh + float(self.rng.uniform(0, 8)), b, float(self.rng.uniform(1, self.r_max)),
                            float(self.rng.uniform(-math.radians(60), math.radians(60))), "false_alarm"))

        # DEDI / MISP filters (PDAT-affecting parameters)
        if p.dedi == 1:
            targets = [t for t in targets if t[1] < 0]
        elif p.dedi == 0:
            targets = [t for t in targets if t[1] > 0]
        if p.misp_pct > 0:
            floor_b = p.misp_pct / 100.0 * (N_BINS // 2)
            targets = [t for t in targets if abs(t[1]) >= floor_b]

        # cap
        cap_hit = len(targets) > p.max_targets
        if cap_hit:
            if p.cap_policy == "weakest_first":
                targets.sort(key=lambda t: -t[0])
            elif p.cap_policy == "fft_order":
                targets.sort(key=lambda t: t[1])
            elif p.cap_policy == "random":
                self.rng.shuffle(targets)
            targets = targets[: p.max_targets]

        # quantise to wire units
        raw: List[RawRadarTarget] = []
        for mag, b, r_rep, az_rep, tag in targets:
            r_q = round((r_rep + self.rng.normal(0, p.range_noise_m)) / self.r_res) * self.r_res
            r_q = max(0.0, r_q)
            az_deg = math.degrees(az_rep) + self.rng.normal(0, p.angle_noise_deg)
            az_deg = max(-90.0, min(90.0, round(az_deg)))
            angle_raw = int(round(-az_deg * 100))          # + = sensor-right = rider's LEFT = -Y
            speed_raw = int(round(b * self.bin_kmh * 100))
            raw.append(RawRadarTarget(distance_cm=int(round(r_q * 100)), speed_raw=speed_raw, angle_raw=angle_raw,
                                      magnitude_raw=int(max(0, min(65535, round(mag * 100))))))

        truth = FrameTruth(t_mid=t_mid, frame_number=fn, vehicles=truth_veh, rider=st, n_scatter_visible=n_visible,
                           n_above_threshold=n_above, n_bins_after_merge=n_after_merge, n_clutter_bins=n_clutter_bins,
                           cap_hit=cap_hit, gap=gap, blocked=blocked, silent=silent)
        if silent or gap:
            return None, truth
        self.seq += 1
        gap_n = 0 if self._last_emitted_fn is None else max(0, fn - self._last_emitted_fn - 1)
        self._last_emitted_fn = fn
        t_header = t_mid + self.T / 2 + p.sensor_delay_s + float(self.rng.uniform(0, p.usb_jitter_s))
        frame = RawRadarFrame(t_header=t_header, frame_number=fn, gap=gap_n, rspi=p.rspi, rrai=p.rrai,
                              targets=tuple(raw), cap_hit=len(raw) >= p.max_targets, source_seq=self.seq)
        return frame, truth

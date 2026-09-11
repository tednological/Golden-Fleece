"""l03: THE ONLY radar unit/sign conversion site.

    +X -> behind the rider      +Y -> rider's RIGHT      -Y -> rider's LEFT      +Z -> up

    d          = distance_cm / 100                  # m
    az_radar   = -radians(angle_raw / 100)          # rad; angle from +X toward +Y
    v_radial   = (speed_raw / 100) / 3.6            # m/s; - = approaching
    v_closing  = -v_radial                          # m/s; + = closing
    x = d cos(az_radar);  y = d sin(az_radar);  z = UNOBSERVED (elevation is an interval)

Read aloud when in doubt: a car on the rider's LEFT gives a POSITIVE K-LD7 angle
and lands at NEGATIVE y.  No theta_kld7 value escapes this module.

    t_mid = t_header - delta_sensor - T_frame(RSPI) / 2
"""
from __future__ import annotations

import math
from typing import List

from ..config import RadarConfig
from ..types import RadarDetection, RadarFrame, RawRadarFrame, StageCounters

STAGE = "l03_radar_decode"


def decode(raw: RawRadarFrame, rcfg: RadarConfig) -> RadarFrame:
    t_frame = rcfg.frame_duration_s[raw.rspi]
    t_mid = raw.t_header - rcfg.sensor_delay_s - t_frame / 2.0
    dets: List[RadarDetection] = []
    rej = {"az_clip": 0, "zero_range": 0}
    for i, tg in enumerate(raw.targets):
        if tg.distance_cm <= 0:
            rej["zero_range"] += 1
            continue
        d = tg.distance_cm / 100.0
        az = -math.radians(tg.angle_raw / 100.0)
        if abs(az) > rcfg.az_clip_rad:
            rej["az_clip"] += 1
            continue
        v_radial = (tg.speed_raw / 100.0) / 3.6
        dets.append(RadarDetection(
            r=d, az=az, v_radial=v_radial, v_closing=-v_radial,
            x=d * math.cos(az), y=d * math.sin(az),
            magnitude_db=tg.magnitude_raw / 100.0, raw_index=i,
            elev_min=rcfg.elev_min_rad, elev_max=rcfg.elev_max_rad,
        ))
    counters = StageCounters(STAGE, n_in=len(raw.targets), n_out=len(dets), rejected=rej)
    return RadarFrame(t_mid=t_mid, t_header=raw.t_header, frame_number=raw.frame_number, gap=raw.gap,
                      rspi=raw.rspi, rrai=raw.rrai, detections=tuple(dets), n_raw=len(raw.targets),
                      cap_hit=raw.cap_hit, counters=counters)

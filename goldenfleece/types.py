"""Inter-layer type contracts.

Frame convention (radar frame, the root):
    +X -> behind the rider      +Y -> rider's RIGHT      -Y -> rider's LEFT      +Z -> up
"forward-left-up" family with +X pointing rearward, so +Y is the rider's RIGHT.

Units: metres, radians, m/s, seconds, unless a field name says ``_raw``, ``_cm``,
``_db``.  Wire-native integers live only in the ``Raw*`` types.  No radar-derived
type carries a ``z`` or an elevation *value*: elevation is an interval.
"""
from __future__ import annotations

import enum
import math
from dataclasses import dataclass, field
from typing import Mapping, Optional, Tuple

import numpy as np

# --- fixed sensor geometry constants (rad) ------------------------------------
AZ_CLIP = math.radians(40.0)          # 0.6981317007977318
ELEV_MIN = -math.radians(17.0)        # -0.29670597283903605
ELEV_MAX = +math.radians(17.0)        # +0.29670597283903605
PDAT_MAX_TARGETS = 12


class RadarTopology(enum.Enum):
    PI_POLLS = "PI_POLLS"            # the only one: there is no MCU to poll the sensor and forward its frames


class SourceTag(enum.Enum):
    DEFINITION = "definition"
    NOMINAL = "nominal"
    MEASURED = "measured"
    CALIBRATED = "calibrated"
    UNVALIDATED = "unvalidated"


@dataclass(frozen=True)
class StageCounters:
    """Every stage reports in / out / rejected-by-reason (rule 4.5)."""
    stage: str
    n_in: int
    n_out: int
    rejected: Mapping[str, int] = field(default_factory=dict)


# --- l01: raw radar -----------------------------------------------------------
@dataclass(frozen=True)
class RawRadarTarget:
    distance_cm: int      # u16
    speed_raw: int        # i16, km/h x 100, positive = receding
    angle_raw: int        # i16, deg x 100, positive = sensor-right (rider's LEFT); confirm on hardware
    magnitude_raw: int    # u16, dB x 100


@dataclass(frozen=True)
class RawRadarFrame:
    t_header: float                       # Clock s: PDAT header arrival, corrected for header transfer + usb latency
    frame_number: int                     # DONE payload
    gap: int                              # frames missed since the previous frame (0 = none)
    rspi: int
    rrai: int
    targets: Tuple[RawRadarTarget, ...]   # <= PDAT_MAX_TARGETS
    cap_hit: bool
    source_seq: int
    resp_code: int = 0


# --- l03: decoded radar --------------------------------------------------------
@dataclass(frozen=True)
class RadarDetection:
    r: float              # m
    az: float             # rad, from +X toward +Y (+ = rider's RIGHT)
    v_radial: float       # m/s, + = receding
    v_closing: float      # m/s, + = closing (= -v_radial)
    x: float              # m
    y: float              # m (+ = rider's RIGHT)
    magnitude_db: float
    raw_index: int
    elev_min: float = ELEV_MIN   # elevation is an interval, never a value
    elev_max: float = ELEV_MAX


@dataclass(frozen=True)
class RadarFrame:
    t_mid: float
    t_header: float
    frame_number: int
    gap: int
    rspi: int
    rrai: int
    detections: Tuple[RadarDetection, ...]
    n_raw: int
    cap_hit: bool
    counters: StageCounters


# --- l02 / l04: IMU -------------------------------------------------------------
class ImuKind(enum.Enum):
    GYRO = "g"
    ACCEL = "a"
    GAME_RV = "q"      # diagnostic only; never fused


@dataclass(frozen=True)
class RawImuSample:
    t: float                          # Clock s
    kind: ImuKind
    values: Tuple[float, ...]         # exactly what the library returned, in its units
    seq: int


class ImuHealth(enum.Enum):
    OK = "OK"
    NO_DATA = "NO_DATA"
    STALE = "STALE"
    RESET = "RESET"
    ACCEL_GATED_LONG = "ACCEL_GATED_LONG"
    INIT_FAILED = "INIT_FAILED"


@dataclass(frozen=True)
class ImuState:
    t: float
    q_level_radar: Tuple[float, float, float, float]   # [w,x,y,z]; roll+pitch only
    q_ref_radar: Tuple[float, float, float, float]     # full attitude in a yaw-arbitrary reference
    omega_radar: Tuple[float, float, float]            # bias-corrected rad/s, radar frame
    gyro_bias: Tuple[float, float, float]
    cov_diag: Tuple[float, ...]                        # diag of the 6x6 error covariance
    yaw_sigma: float                                   # grows without bound (unobservable)
    roll: float
    pitch: float
    in_motion: bool
    motion_energy: float
    health: ImuHealth
    n_gyro: int
    n_accel_applied: int
    n_accel_gated: int


# --- l05: ego motion ------------------------------------------------------------
class EgoInvalidReason(enum.Enum):
    NONE = "NONE"
    NO_CANDIDATES = "NO_CANDIDATES"
    TOO_FEW_INLIERS = "TOO_FEW_INLIERS"
    LOW_AZ_SPREAD = "LOW_AZ_SPREAD"
    POOR_FIT = "POOR_FIT"
    MERGED_BINS_SUSPECTED = "MERGED_BINS_SUSPECTED"
    NO_IMU = "NO_IMU"


class EgoSource(enum.Enum):
    RADAR_RANSAC = "RADAR_RANSAC"
    IMU_STOPPED = "IMU_STOPPED"
    NONE = "NONE"


@dataclass(frozen=True)
class EgoMotion:
    t: float
    v_s: Tuple[float, float]              # sensor velocity in the radar frame (x, y), m/s
    cov: Tuple[Tuple[float, float], Tuple[float, float]]
    speed: float
    inlier_mask: Tuple[bool, ...]
    n_candidates: int
    n_inliers: int
    az_spread_rad: float
    valid: bool
    invalid_reason: EgoInvalidReason
    source: EgoSource
    # DIAGNOSTIC ONLY: vest-to-travel yaw.  Logged, never consumed downstream.
    psi_travel: Optional[float]
    psi_travel_sigma: Optional[float]
    counters: StageCounters


# --- l06: clutter ----------------------------------------------------------------
class DetectionClass(enum.Enum):
    STATIONARY = "STATIONARY"
    RECEDING_MOVER = "RECEDING_MOVER"
    RECEDING_UNCLASSIFIED = "RECEDING_UNCLASSIFIED"   # ego-motion invalid
    APPROACHING = "APPROACHING"


@dataclass(frozen=True)
class ClassifiedDetection:
    det: RadarDetection
    cls: DetectionClass
    ego_valid: bool


@dataclass(frozen=True)
class ClutterOutput:
    t_mid: float
    frame_number: int
    kept: Tuple[ClassifiedDetection, ...]
    cap_hit: bool
    counters: StageCounters


# --- l07: tracking ---------------------------------------------------------------
class TrackStatus(enum.Enum):
    TENTATIVE = "TENTATIVE"
    CONFIRMED = "CONFIRMED"
    COASTING = "COASTING"
    DELETED = "DELETED"


class CoastReason(enum.Enum):
    NONE = "NONE"
    DOPPLER_BLIND = "DOPPLER_BLIND"
    FOV_EXIT = "FOV_EXIT"
    YAW_EXIT = "YAW_EXIT"
    UNEXPLAINED = "UNEXPLAINED"


class CoastResolution(enum.Enum):
    NONE = "NONE"
    PASS_THROUGH_COMPLETE = "PASS_THROUGH_COMPLETE"
    SIGMA_BOUND_EXCEEDED = "SIGMA_BOUND_EXCEEDED"
    REACQUIRED = "REACQUIRED"
    NEVER_CONFIRMED = "NEVER_CONFIRMED"     # tentative track dropped; contributed no threat


@dataclass(frozen=True)
class Track:
    id: int
    status: TrackStatus
    coast_reason: CoastReason
    coast_resolution: CoastResolution
    t: float
    r: float
    r_dot: float                 # + = receding
    sigma_r: float
    sigma_r_dot: float
    az: float                    # level frame
    sigma_az: float
    x: float
    y: float
    v_closing: float             # = -r_dot
    hits: int
    misses: int
    age: int
    frames_since_update: int
    coast_started_t: Optional[float]
    last_class: DetectionClass
    n_merged_measurements: int
    closing: bool                # r_dot < -blind band at last update / prediction
    kinematic_consistent: bool   # range trend agrees with range-rate (wheel micro-Doppler phantoms fail this)


@dataclass(frozen=True)
class TrackerOutput:
    t_mid: float
    frame_number: int
    tracks: Tuple[Track, ...]
    counters: StageCounters


# --- l08: threat ---------------------------------------------------------------------
class ThreatLevel(enum.IntEnum):
    NONE = 0
    ADVISORY = 1
    WARNING = 2
    ALERT = 3


class Side(enum.Enum):
    LEFT = "LEFT"
    CENTER = "CENTER"
    RIGHT = "RIGHT"
    BOTH = "BOTH"
    NONE = "NONE"


@dataclass(frozen=True)
class ThreatAssessment:
    track_id: int
    level: ThreatLevel
    side: Side
    t_arrival: float            # time to arrival at the rider's LONGITUDINAL position; not time to collision
    t_arrival_low: float
    r: float
    v_closing: float
    confidence: float
    coasting: bool
    coast_reason: CoastReason
    proximity_triggered: bool


# --- health ---------------------------------------------------------------------------
class HealthState(enum.IntEnum):
    OK = 0
    DEGRADED = 1
    OFFLINE = 2


class HealthBits(enum.IntFlag):
    NONE = 0
    RADAR_SILENT = 1 << 0
    RADAR_GAPS = 1 << 1
    RADAR_POSSIBLY_BLOCKED = 1 << 2
    PDAT_SATURATED = 1 << 3
    IMU_FAULT = 1 << 4
    EGO_INVALID = 1 << 5
    UNDERVOLTAGE = 1 << 6
    THROTTLED = 1 << 7
    PIPELINE_RESTARTING = 1 << 8
    HAPTICS_FAULT = 1 << 9          # the vibration motors cannot be driven: the rider gets no warnings
    RADAR_CONFIG_MISMATCH = 1 << 10


OFFLINE_BITS = (HealthBits.RADAR_SILENT | HealthBits.PIPELINE_RESTARTING | HealthBits.RADAR_CONFIG_MISMATCH
                | HealthBits.HAPTICS_FAULT)
DEGRADED_BITS = (HealthBits.RADAR_GAPS | HealthBits.RADAR_POSSIBLY_BLOCKED | HealthBits.PDAT_SATURATED
                 | HealthBits.IMU_FAULT | HealthBits.UNDERVOLTAGE | HealthBits.THROTTLED)
INFORMATIONAL_BITS = HealthBits.EGO_INVALID


@dataclass(frozen=True)
class HealthEvent:
    t: float
    bit: HealthBits
    active: bool
    source: str
    detail: str = ""


@dataclass(frozen=True)
class RadarConfigChanged:
    t: float
    rrai: int
    rspi: int
    baud: int
    thof: int
    dedi: int
    misp: int
    masp: int
    firmware_version: str
    frame_duration_s: float


# --- l09: warning command -------------------------------------------------------------
class TArrivalBucket(enum.IntEnum):
    NONE = 0
    GT_6S = 1
    S3_6 = 2
    S1P5_3 = 3
    LT_1P5 = 4


@dataclass(frozen=True)
class WarningCommand:
    seq: int
    t_decided: float
    level: ThreatLevel
    side: Side
    t_arrival_bucket: TArrivalBucket
    health_state: HealthState
    health_bits: HealthBits
    assert_alert: bool                 # the alert channel; only ever true for ThreatLevel.ALERT
    dominant_track_id: Optional[int]
    flicker_count: int


# --- transforms ----------------------------------------------------------------------------
@dataclass(frozen=True)
class Transform:
    """T_target_source maps a point from ``source`` into ``target``."""
    target: str
    source: str
    q_wxyz: Tuple[float, float, float, float]
    t_xyz: Tuple[float, float, float]
    source_tag: SourceTag


def as_vec3(v) -> np.ndarray:
    a = np.asarray(v, dtype=float).reshape(3)
    return a

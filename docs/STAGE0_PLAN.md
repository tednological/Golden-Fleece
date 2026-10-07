# Golden Fleece — Stage 0 plan (report, then stop)

> **Historical.** Since 2026-10-07 there is no MCU (team decision 12, `docs/DECISIONS.md`): the vibration motors are on
> the Pi's GPIOs and l10 drives them directly (`docs/haptics.md`). The MCU, its link and its ICD below describe the
> design as planned then.

Parameters in effect: `RADAR_TOPOLOGY = PI_POLLS`, `MCU_TRANSPORT = USB_CDC`.
Inputs used: the task prompt and `docs/K-LD7_Datasheet.pdf` (RFbeam, Revision B, 03/2021, 23 pages). Nothing fetched.

---

## 0. Working directory, environment, existing material

**Existing content (left untouched, not read):**

| Item | Action |
|---|---|
| `1_radar_data_input/ … 11_web_app/` (11 numbered directories, all empty) | Left as is. The new package uses the §3 layout (`goldenfleece/l01_…`), not these directories. |
| `run.sh` (57 bytes) | Not read, not run, not modified. |
| `docs/K-LD7_Datasheet.pdf` | Read in full (the authoritative sensor reference). |
| `docs/The KLD7 Python API — kld7 0.2.1 documentation.pdf` | **Not read.** It documents a third-party K-LD7 library, which rule §4.6 forbids. A `kld7` package is also installed system-wide in this Python; it will never be imported, and an invariant test greps for it. |

**Environment facts (measured on this machine):**

| Fact | Value | Consequence |
|---|---|---|
| Host | Raspberry Pi 5 Model B Rev 1.1, kernel 6.12.75, Python 3.11.2 | as specified |
| Present packages | numpy 1.24.2, pyserial 3.5, PyYAML 6.0, gpiod 1.6.3 | gpiod is the **v1 API** (`Chip`/`Line`), not v2; l10 targets v1 |
| Missing packages | scipy, pytest, hypothesis, adafruit-circuitpython-bno08x, adafruit-blinka | to install in a project venv at Stage 1 (see §10) |
| USB serial devices | `/dev/serial/by-id/usb-Prolific_…CSDNb151406-if00-port0` (PL2303, `ttyUSB0`) and `/dev/serial/by-id/usb-Microchip…MCP2221…-if00` (`ttyACM0`) | Which one carries the K-LD7 is a question (§9). Achieved baud is measured by the probe. |
| SPI | `dtparam=spi=on` absent; no `/dev/spidev0.*` | BNO085 bring-up needs SPI0 enabled (checklist item, Stage 5) |
| GPIO boot pulls (via `pinctrl`) | GPIO17 = pull-down, GPIO26 = pull-down, GPIO4 = pull-up | GPIO17 proposed for the alert line: idle LOW, asserted HIGH |
| `vcgencmd get_throttled` | `0x0`, tool present | power monitor feasible as specified |
| Storage | one 238 GB `mmcblk0` (root); no second disk | recording path question (§9) |
| Git | not a repository; global identity configured | initialised at the end of Stage 0 to commit this plan |
| `isolcpus` | not set | deployment doc only, opt-in |

---

## 1. Datasheet cross-check (attached Rev B vs prompt §8)

Every §8 statement was checked against the datasheet. Confirmed verbatim: polled GNFD model (Fig. 14), packet format (Table 9), RESP codes (Table 13), boot baud 115200 8E1 (Table 8), INIT baud indices 0–4 (Table 14), GNFD bit-field (Table 14), PDAT layout and 0–96 byte length (Table 13 → 12 targets), speed sign (p.6: positive = receding), point-cloud behaviour (p.7), TDAT single target (p.7, p.10), RSPI table (Table 4), RRAI table (Table 3), the PDAT-filter table (Table 6: MISP/MASP and DEDI affect PDAT; MIRA/MARA/MIAN/MAAN and thresholds affect DDAT only; THOF affects raw targets per p.10), parameter defaults differ from operational (Table 12), and the 125-byte worst-case wire cost.

**Differences, errata, and additions (none changes a requirement; §15.2 not triggered):**

| # | Topic | Datasheet says | Effect |
|---|---|---|---|
| 1 | **Erratum, Fig. 19** | The GBYE example bytes are `49 4E 49 54` = "INIT", not "GBYE" (`47 42 59 45`) | Driver sends ASCII `GBYE` per Table 11. Test vector uses the corrected bytes and cites the erratum. |
| 2 | Baud reset | "The sensor always starts up with its default baud rate" (p.13) | Power-cycle also returns to 115200, not only GBYE. Reconnect still probes all rates (process death without GBYE leaves the sensor high). |
| 3 | Baud-change sequencing | RESP to INIT is sent **before** the switch; RESP to GBYE before reverting (p.13) | Driver waits for RESP at the old rate, then reconfigures the port and flushes. |
| 4 | Angle sign | Text: "a positive or negative angle defines if the target is more on the right or left" (does not say which). Table 2/7 + DDAT: angle > ANTH (default 0°) ⇒ angle flag = 1 = "Right". Fig. 5: field of view drawn in front of the board with 0° toward the viewer, +90° at the viewer's left, which is the **sensor's right** when it looks outward. | Consistent with "+ = sensor-right". No negation added anywhere; physical confirmation remains Stage 6. |
| 5 | SRPS needs a version match | RESP 3 = "Invalid RPST version"; the 42-byte structure begins with the 19-byte software-version string | Connect sequence must be INIT → **GRPS** → build SRPS with the returned version → SRPS → GRPS → compare. Writing "the full structure" requires reading it first. |
| 6 | **Raw targets are Doppler-FFT bins** | Fig. 4: "search all targets above a threshold in the FFT", 256-point complex FFT, one distance and one angle computed per target from phase differences | At most one raw target per Doppler bin (0.78 km/h = 0.217 m/s at RSPI 3). Scatterers sharing a bin **merge** into one target whose distance and angle come from the composite phase. §8 does not mention this. It drives the cap estimate (§5), the ego-motion design (§4.5) and the simulator model (§4.12). |
| 7 | Frame period origin | 256 samples at twice the 100 km/h Doppler (4.47 kHz) = 28.6 ms | Confirms the 29 ms typical; the frame is a genuine 29 ms integration window (t_mid formula is appropriate). |
| 8 | DONE payload type | "Frame number since reset", 4 bytes, datatype not stated | Assume UINT32 LE (consistent with every other multi-byte field). |
| 9 | RESP 6 | "Timeout error" condition not defined | Treated as a generic command failure; counted by code. |
| 10 | >12 targets above threshold | Not addressed | Truncation order (strongest first? bin order?) is unknown. Simulator drops weakest first as §13.1 says, tagged unverified. Probe measures cap-hit rate only. |
| 11 | Speed aliasing | Targets above the speed setting "can generate wrong measurements" (p.9) | At RSPI 3, |v_rel| > 27.8 m/s aliases (a 110 km/h car passing a stopped rider can appear receding). No software fix; recorded as a limitation. |
| 12 | Radome rule | Cover ≥ 6.2 mm from the sensor face; non-metallic; **antenna vibrating relative to the cover generates false signals** (p.20) | Vest fabric over the radar is a radome. Fabric flap can create false near targets, and a blocked sensor may not look "empty". Affects the blockage heuristic (§4.9) and hardware. |
| 13 | Rx spacing | 6.223 mm = 0.50 λ at 24.15 GHz | Phase-monopulse angle is unambiguous over ±90°, as §6.1 assumes. |
| 14 | "Never transmits unasked" | Not stated; implied by the client–server model | Driver tolerates and counts unexpected packets anyway (resync path). |

**Datasheet-provided test vectors (turned into parser tests verbatim):** Fig. 17 INIT/RESP bytes, Fig. 18 GNFD/RESP/TDAT bytes, Fig. 19 RESP bytes (with the header erratum noted), Table 15 TDAT conversion (0x0050 → 80 cm, 0xFF97 → −1.05 km/h, 0x072F → 18.39°, 0x1815 → 61.65 dB). The TDAT vector exercises the 8-byte target layout only; TDAT is never requested.

**RPST (42 bytes) layout, from Table 12, as the driver packs/unpacks it:**
`char[19] version (NUL-terminated) | u8 base_freq | u8 max_speed(RSPI) | u8 max_range(RRAI) | u8 thof | u8 trft | u8 visu | u8 mira | u8 mara | i8 mian | i8 maan | u8 misp | u8 masp | u8 dedi | u8 rath | i8 anth | u8 spth | u8 dig1 | u8 dig2 | u8 dig3 | u16 hold | u8 mide | u8 mids` = 19 + 19 + 2 + 2 = 42.

---

## 2. Dataclass contracts (all in `goldenfleece/types.py`, frozen dataclasses, SI unless the name says `_raw`/`_cm`)

Every module that touches geometry carries the header comment:
`# +X → behind the rider   +Y → rider's RIGHT   −Y → rider's LEFT   +Z → up`

```
RawRadarTarget      distance_cm:int(u16)  speed_raw:int(i16, km/h×100, + = receding)
                    angle_raw:int(i16, deg×100, + = sensor-right, unconfirmed)  magnitude_raw:int(u16, dB×100)

RawRadarFrame       t_header:float   # Clock s; PDAT-header arrival minus 4-byte header transfer time minus usb_latency_s
                    frame_number:int (DONE)   gap:int (frames missed since previous; 0 = none)
                    rspi:int  rrai:int (in effect)   targets:tuple[RawRadarTarget,...] (≤12)
                    cap_hit:bool (len==12)   source_seq:int   resp_code:int
                    NOTE: wire-native integers only; no floats derived from them (rule 4.1)

RadarDetection      r:float(m)  az:float(rad, from +X toward +Y)  v_radial:float(m/s, + receding)
                    v_closing:float(= −v_radial)  x:float  y:float  magnitude_db:float
                    elev_min:float = −0.296706  elev_max:float = +0.296706   # interval; NO z field
                    raw_index:int

RadarFrame          t_mid:float  t_header:float  frame_number:int  gap:int  rspi:int  rrai:int
                    detections:tuple[RadarDetection,...]  n_raw:int  cap_hit:bool
                    counters:StageCounters   # in, out, rejected{az_clip, range_zero, ...}

RawImuSample        t:float (Clock s at INT edge or read)   kind: GYRO|ACCEL|GAME_RV(diag only)
                    values:tuple[float,...]  exactly as the library returns them (its units), untouched
                    seq:int   lib_status:int|None

ImuState            t:float   q_level_radar:(w,x,y,z)  (roll+pitch only)   q_ref_radar:(w,x,y,z) (full attitude in a yaw-arbitrary reference; only differences of its yaw are used)
                    omega_radar:(3,) bias-corrected rad/s in radar frame   gyro_bias:(3,)
                    cov:(6,6)  yaw_sigma:float (grows unbounded)   in_motion:bool  motion_energy:float
                    health:ImuHealth {OK, NO_DATA, STALE, RESET, ACCEL_GATED_LONG}   counters:StageCounters
ImuStateProvider    .state_at(t)->ImuState (interpolated)   .delta_yaw(t1,t2)->float rad   .at_or_none

EgoMotion           t:float  v_s:(2,) m/s radar frame  cov:(2,2)  speed:float
                    inlier_mask:tuple[bool,...]  n_candidates:int  n_inliers:int  az_spread_rad:float
                    valid:bool  invalid_reason:EgoInvalidReason {NONE, NO_CANDIDATES, TOO_FEW_INLIERS, LOW_AZ_SPREAD, POOR_FIT, MERGED_BINS_SUSPECTED, RIDER_STOPPED_IMU}
                    psi_travel:float|None  psi_travel_sigma:float|None   # DIAGNOSTIC ONLY, never consumed downstream

DetectionClass      STATIONARY | RECEDING_MOVER | APPROACHING | RECEDING_UNCLASSIFIED (ego invalid)
ClassifiedDetection det:RadarDetection  cls:DetectionClass  ego_valid:bool
ClutterOutput       t_mid, frame_number, kept:tuple[ClassifiedDetection,...], cap_hit:bool,
                    counters (in, stationary_dropped, receding_mover, receding_unclassified, approaching, out)

TrackStatus         TENTATIVE | CONFIRMED | COASTING | DELETED
CoastReason         NONE | DOPPLER_BLIND | FOV_EXIT | YAW_EXIT | UNEXPLAINED
CoastResolution     NONE | PASS_THROUGH_COMPLETE | SIGMA_BOUND_EXCEEDED | REACQUIRED
Track               id:int  status  coast_reason  coast_resolution  t:float
                    r:float  r_dot:float  sigma_r:float  sigma_r_dot:float        # range/range-rate KF
                    az:float  sigma_az:float                                     # level-frame azimuth KF
                    x:float  y:float  v_closing:float (= −r_dot)                  # derived, no z
                    hits:int  misses:int  age:int  frames_since_update:int  coast_started_t:float|None
                    last_class:DetectionClass  n_merged_measurements:int
TrackerOutput       t_mid, frame_number, tracks:tuple[Track,...], counters (in, clustered, assigned, gated_out, spawned, confirmed, coasting_by_reason{...}, deleted_by_resolution{...})

ThreatLevel         NONE=0 | ADVISORY=1 | WARNING=2 | ALERT=3
Side                LEFT | CENTER | RIGHT | BOTH
ThreatAssessment    track_id, level, side, t_arrival:float, t_arrival_low:float, r, v_closing,
                    confidence:float(0..1), coasting:bool, coast_reason, proximity_triggered:bool

HealthState         OK | DEGRADED | OFFLINE
HealthBits(IntFlag) RADAR_SILENT=1 RADAR_GAPS=2 RADAR_POSSIBLY_BLOCKED=4 PDAT_SATURATED=8 IMU_FAULT=16
                    EGO_INVALID=32 UNDERVOLTAGE=64 THROTTLED=128 PIPELINE_RESTARTING=256 MCU_LINK_DOWN=512
HealthEvent         t, bit:HealthBits, active:bool, source:str, detail:str
TArrivalBucket      NONE | GT_6S | S3_6 | S1P5_3 | LT_1P5
WarningCommand      seq:int  t_decided:float  level:ThreatLevel  side:Side  t_arrival_bucket
                    health_state:HealthState  health_bits:HealthBits  assert_alert_line:bool  dominant_track_id:int|None

RadarConfigChanged  t, rrai, rspi, baud, thof, dedi, misp, masp, firmware_version:str, frame_duration_s:float
StageCounters       stage:str  n_in:int  n_out:int  rejected:Mapping[str,int]
Transform           target:str  source:str  q_wxyz:(4,)  t_xyz:(3,)  source_tag:{definition,nominal,measured,calibrated}
```

Invariant 3 is enforced structurally: no radar-derived dataclass has a `z` or `elev` scalar field, and a test walks `types.py` and every layer's output types to assert it.

---

## 3. Shared modules

- **`clock.py`** — `Clock` protocol: `now() -> float` (seconds), `sleep(dt)`. `MonotonicClock` uses `time.clock_gettime(time.CLOCK_MONOTONIC)`. `FakeClock` has `advance(dt)` and `set(t)`. The only file allowed to import `time`. Adapters wait on I/O with pyserial/gpiod timeouts (I/O, not clocks); the grep test allows `timeout=` arguments but not `time.*` calls.
- **`frames.py`** — quaternion utilities (`[w,x,y,z]`), `Transform`, `compose(A,B)` raising `FrameMismatchError` when `A.source != B.target`, `invert`, `apply`, `to_matrix`, `euler_zyx_for_display`. Config loader validates orthonormality, det = +1, and the mandatory `source` tag.
- **`config.py`** — typed loading of the three YAML files; the unmeasured-parameter register with tags `nominal | measured | calibrated | unvalidated`; refuses to start on a required `null` unless `allow_unmeasured: true`, which logs loudly on every start.
- **`health.py`** — `HealthBits` aggregation to `HealthState`; onset timestamps per bit; the documented worst-case onset-to-MCU table is checked by a test that injects each fault into the simulator harness and measures the time to the emulator.

---

## 4. Per-layer design and justification

### 4.1 `l01_radar_data_input` — driver structure
- `RadarFrameSource` protocol: `start()`, `stop()`, `get(timeout_s) -> RawRadarFrame|None` (bounded queue, newest-wins), `events()` (health + `RadarConfigChanged`), `stats()`. Under `PI_POLLS` only `DirectKld7Source` is built. `McuForwardedSource` is **not** built in this task (topology is fixed by the parameter); the protocol and the ICD leave room for it.
- **Pure parser** `Kld7Parser.feed(bytes) -> list[Packet]`: header scan over the known 4-char set, length sanity (≤ 3072), resync by sliding one byte on anything invalid, counters for resyncs. Fully unit-tested with the datasheet vectors and with fuzzing (hypothesis).
- **Reader thread** owns the port and runs the state machine: `DISCONNECTED → PROBING → INIT → READ_PARAMS → WRITE_PARAMS → VERIFY → STREAMING`, with `BACKOFF` (exponential, 0.25–4 s) on any fault. Faults are `HealthEvent`s: RESP codes 1–6 (each counted), resync events, frame-number gaps, port loss (reconnect via the by-id path), `RADAR_SILENT` after `k · T_frame(RSPI)` (k = 5 → 145 ms at RSPI 3) without a valid DONE.
- **Connect:** open at the configured rate; **probe** = send `GBYE`, expect `RESP` within 100 ms; on failure try 115200, 460800, 921600, 2000000, 3000000 in turn (each with its own port reconfigure). After a good GBYE the sensor is at 115200: open at 115200 → `INIT(idx)` → wait RESP at 115200 → reconfigure to the new rate → `GRPS` → fill our fields into the returned structure (keeping the version string) → `SRPS` → `GRPS` → byte-compare; refuse to stream on mismatch (fault event, no silent fallback). Log the version string; emit `RadarConfigChanged`. Clean shutdown sends `GBYE`.
- **Poll cycle:** `GNFD(0x24)` → `RESP(0)` → `PDAT` → `DONE`. `t_header` is stamped at the arrival of the first PDAT byte (RESP arrives before acquisition and is useless for timing), corrected by 4 × 11 / baud and by `usb_latency_s` from config. The next `GNFD` is sent immediately on `DONE`. The probe additionally tests early polling (GNFD sent right after RESP) to see whether the sensor queues it or answers RESP 5; the result decides whether the driver pipelines polls.
- **Baud = 921600 (config default), justification:** the frame body (RESP + PDAT + DONE ≤ 125 B) costs 11.9 ms at 115200 and **varies with target count** (2.9–11.9 ms), versus 1.5 ms at 921600. That variable 9 ms is ~20 % of the total budget (§6) and it lands on the path after the timestamp, so it is pure latency. 2–3 Mbaud gains under 1 ms more, needs an adapter that supports 8E1 at that rate, and is the datasheet's recommendation only for RADC/RFFT. 921600 is the highest rate that common USB-TTL bridges support with even parity; if the attached bridge cannot (the MCP2221 likely tops out at 460800), the probe reports it and config drops to 460800 (3.0 ms), which still beats 115200 by 9 ms.
- **Probe script** `tools/kld7_probe.py` logs: version string; per-frame target count and cap-hit rate; frame-number gaps; measured frame period per RSPI (from DONE numbers and header timestamps); achieved baud; GNFD→PDAT-header delay distribution (its **minimum estimates δ_sensor** if acquisition is free-running; its mean minus T_frame estimates it if acquisition is triggered by the poll); header-timestamp jitter; the early-poll experiment.

### 4.2 `l02_imu_data_input`
- Blinka `board.SPI()` + `adafruit_bno08x.spi.BNO08X_SPI(spi, cs, int_pin, reset_pin)`. A reader thread waits on the INT line (gpiod edge event where the library allows the caller to wait; otherwise the library's own INT polling), stamps `t = clock.now()` as close to the INT edge as the library allows, drains reports, and pushes `RawImuSample`s to a ring buffer. Emits health: `NO_DATA` (> 100 ms without a sample), `RESET` (library reports a reset; re-enable features), `INIT_FAILED`.
- Reports: raw gyroscope and raw accelerometer only. Proposed 200 Hz gyro and 100 Hz accelerometer (gyro higher because Δψ during a 150 °/s shoulder check must be integrated with < 0.5° error per 29 ms frame; accelerometer only steers roll/pitch slowly). The library's default report interval is a fixed value that I expect to be far below 100 Hz; I will use the library's own feature-report mechanism to set the interval (inspecting the installed library source is in scope — it is the mandated library, not a protocol doc). **If the measured rate < 100 Hz gyro or the timestamp jitter p99 > 5 ms, I stop and report (§9.2, §15.4).** No SH-2 driver from memory.
- Game rotation vector: optional diagnostic report at 20 Hz, logged, never fused.

### 4.3 `l03_radar_decode`
Exactly §6.2. `t_mid = t_header − δ_sensor − T_frame(RSPI)/2` from config. Rejections counted by reason (`az_clip`, `zero_range`). Golden vectors §6.3 verified numerically during this stage: (2000, +863) → az −0.150622, x 19.7736, y −3.0011; (3000, −2000) → az +0.349066, x 28.1908, y 10.2606; ±5400 → ∓15.0 m/s; 4000 accepted, 4001 and 5500 rejected.

### 4.4 `l04_imu_decode` — ESKF
- **SI conversion:** the mandated library already returns rad/s and m/s²; the conversion step is an explicit identity with the library's units declared, verified by the +9.81 m/s² on +Z test (§5.5).
- **Extrinsic:** `T_radar_imu` from `frames.yaml`, validated orthonormal, det +1; axis map applied here only.
- **Nominal state:** attitude quaternion `q_ref_radar` (radar attitude in a yaw-arbitrary, gravity-aligned reference) + gyro bias (3). **Error state:** δθ (3), δb_g (3) → 6×6 covariance. **Accelerometer bias not estimated**: with only a gravity-direction update it is weakly observable without excitation, and a typical few-mg bias tilts the estimate by < 0.3°, which is far inside the pitch-foreshortening and yaw floors of §7. Gyro bias is kept because it is cheap and it bounds yaw drift over track lifetimes.
- **Propagate** on each gyro sample (dt from sample stamps). **Update** with the accelerometer as a gravity-direction measurement, **gated**: `| ‖a‖ − g | < 0.5 m/s²` and `‖ω‖ < 0.35 rad/s` (config, `unvalidated`). Gating handles the coordinated-turn degeneracy; if gated for > 5 s the state reports `ACCEL_GATED_LONG` (informational).
- **Centripetal compensation** (`enabled: false` by default): `a_c = ω × v_ego` with `v_ego` = the ego velocity published by l05 on the **previous** frame, carried in by the orchestrator explicitly as `prev_ego`; l04 never calls l05.
- **Yaw:** no update ever touches yaw; its variance grows with gyro noise + bias variance. `delta_yaw(t1,t2)` = difference of the reference yaw of `q_ref_radar` at the two times (unwrapped). `q_level_radar` = `q_ref_radar` with its yaw removed.
- **Rider in motion:** rolling 1 s variance of `‖a‖` and `‖ω‖` above thresholds (config).

### 4.5 `l05_ego_motion`
- Model `v_radial,i = −(v_s · u_i)` with `u_i = (cos az_i, sin az_i)`; RANSAC over 2-point minimal sets (exact 2×2 solve), inlier threshold 1.5 speed bins + an angle-noise term, least-squares refit on the consensus set, covariance from residuals.
- **Validity:** ≥ 3 inliers, azimuth spread ≥ 15°, RMS residual < threshold, and consistency with a **top-of-band cross-check**: the largest receding `v_radial` among candidates approximates rider speed independently of angle quality; if the fitted speed disagrees by > 20 % the estimate is invalid with `MERGED_BINS_SUSPECTED` (see datasheet item 6: symmetric scenes merge ±az scatterers into bins with ~0° reported angle, biasing a naive fit low). The previous estimate ± IMU-integrated speed change is used as a prior gate against a slower same-direction vehicle capturing the consensus.
- Degenerate cases: rider stopped (l04 `in_motion = False` → `v_s = 0`, valid, reason `RIDER_STOPPED_IMU`; the radar cannot see stationary clutter at rest); open road (`NO_CANDIDATES` / `TOO_FEW_INLIERS`, invalid); slower same-direction vehicle (prior gate + top-of-band check).
- **Diagnostic** `ψ_travel = atan2(−v_sy, −v_sx)` with σ from the covariance: computed, logged, reported on synthetic data, consumed by nothing.
- Golden vectors: `v_s = (−5, 0)`: pole behind → `v_radial = +5.0`, `v_closing = −5.0`; pole at 30° → `+4.330`.

### 4.6 `l06_clutter_rejection`
- Approaching (`v_closing > blind band`) → `APPROACHING`, **always kept**, regardless of `EgoMotion.valid`. Receding with ego valid: residual against the ego model < threshold → `STATIONARY` (dropped, counted); else `RECEDING_MOVER`. Receding with ego invalid → `RECEDING_UNCLASSIFIED`.
- **Decision (for approval):** receding movers and unclassified receding detections **do** reach the tracker, labelled. Reason: the same-speed pacer drifts across the blind band; a track that survives the zero crossing warns immediately when the pacer starts closing again, whereas a fresh track needs M-of-N frames. Cost: more tentative tracks (bounded by 12 detections/frame); the tracker applies stricter confirmation (4-of-6) to non-approaching tracks; threat ignores non-closing tracks.
- Cap monitoring: `PDAT_SATURATED` when ≥ 50 % of frames in a 1 s window hit 12 (config). Simulator quantifies contention (§5).

### 4.7 `l07_tracking` — decoupled range/range-rate + azimuth
- **Why decoupled, not Cartesian CV:** the Doppler measurement **is** the range rate `ṙ` (relative, ego-inclusive) and range is direct, so `(r, ṙ)` is a *linear* 2-state KF with two direct measurements and constant-`ṙ` dynamics; `t_arrival = r / (−ṙ)` and its uncertainty fall straight out. Azimuth is weak (1° quantisation, ±15° yaw floor) and is a separate 1-state KF whose prediction is rotated by `−Δψ` from l04 each frame (shoulder checks: 3–6° per frame). A Cartesian CV state would couple the weak azimuth into `vx, vy` and make `t_arrival` uncertainty depend on the poorly observed lateral velocity. Cartesian `(x, y)` are derived for output only.
- **Pre-clustering** of raw detections (single link: Δr < 2 m, Δṙ < 1.5 m/s, Δaz < 8°; magnitude-weighted; R inflated by spread) turns a car's point cloud into one measurement; counted.
- **Assignment:** Hungarian (`scipy.optimize.linear_sum_assignment`) on Mahalanobis cost in `(r, ṙ, az)` with block-diagonal innovation covariance; gate χ²₃ < 11.34. Dummy columns for "no assignment".
- **Lifecycle:** `TENTATIVE → CONFIRMED` (3-of-4 approaching, 4-of-6 otherwise) `→ COASTING → DELETED`. Coast reason chosen at the first miss: `DOPPLER_BLIND` if `|ṙ| < blind + margin`; `FOV_EXIT` if `|az| > AZ_CLIP − margin` and `ṙ < 0`; `YAW_EXIT` if the Δψ-rotated prediction lies outside ±AZ_CLIP; otherwise `UNEXPLAINED`. Coast dynamics per §9.7: DOPPLER_BLIND bounds `|ṙ| ≤ blind band` and grows `σ_r` at that rate; FOV_EXIT predicts pass-through (`t_pass = r cos(az) / |ṙ|` + hold) and holds; YAW_EXIT predicts with the gyro and re-acquires; UNEXPLAINED uses high process noise and a flag.
- **Deletion only by resolution:** `PASS_THROUGH_COMPLETE`, `SIGMA_BOUND_EXCEEDED`, or `REACQUIRED` (back to CONFIRMED). No track is deleted because measurements stopped. Tentative tracks that never confirm are dropped (they never contributed a threat).
- The tracker takes `Δψ` and `T_level_radar` as inputs; nothing about the mount is hard-coded.

### 4.8 `l08_threat`
- Only closing tracks (`ṙ < −blind`) are assessed. `t_arrival = r / v_closing`; `t_arrival_low = (r − kσ_r) / (v_closing + kσ_v)`, k = 2 (config). Levels by `t_arrival_low` thresholds (placeholders 6 / 3 / 1.5 s) and the proximity rule (`r < r_close`, placeholder 4 m) → at least WARNING. Coasting tracks keep contributing with inflated σ (UNEXPLAINED coasts hold their last level rather than escalating on inflated σ — a tuning decision to be tested).
- **Side:** lateral offset `y = r sin(az)`; CENTER half-width = `y_c0 + r · tan(σ_ψ)` with `σ_ψ` = 15° floor, so it widens with range. **Confidence** decreases with range and halves while coasting. No lane vocabulary anywhere (grep test).

### 4.9 `l09_warning_policy` and health
- Arbitration: max level; side from the dominant track (highest level, then nearest); `BOTH` if two tracks at the max level disagree. Escalate immediately; de-escalate only after dwell (1.0 s) with hysteresis (×1.3 on the time thresholds) and a minimum on-time (0.7 s at WARNING+). Flicker = level changes per minute, logged. De-escalation happens only when assessments end by coast resolution (structural, from l07) and never on disappearance.
- Health: separate channel; `OFFLINE` if any OFFLINE bit, `DEGRADED` if any DEGRADED bit. `RADAR_POSSIBLY_BLOCKED`: rider in motion (l04) and, for > 5 s (config, unvalidated), zero raw targets and zero ego inliers → DEGRADED, rendered as an advisory. The datasheet's radome warning means a fabric-covered sensor may show flapping false targets instead of emptiness; the heuristic will not catch that case — documented, measured in the simulator's open-road runs for the false-alarm rate.
- **Alert line:** asserted for `ALERT` only; never for any fault (test).

### 4.10 `l10_mcu_link`
- Framing: `[ver=1][type][seq u8][payload][crc16 CCITT-FALSE, big-endian]` → COBS → `0x00` delimiter. CRC and COBS implemented in-package (no extra dependency).
- Writer thread owns the port; bounded queue (depth 8); `WARNING` and `HEARTBEAT` coalesce (newest wins); the hot path only `put_nowait`s. USB re-enumeration → `MCU_LINK_DOWN` (Pi-side health, informational) → reopen by by-id path with backoff.
- GPIO: libgpiod v1 (`Chip` by label `pinctrl-rp1`, `Line.request(consumer, LINE_REQ_DIR_OUT)`), asserted in the same call path **before** the frame is queued; released on exit/signal so the line returns to its pull-down idle.
- Heartbeat: built **inside the orchestrator loop** with `loop_iter`, `frames_processed`, `last_frame_number`, health; l10 has no timer.

### 4.11 Orchestrator
- Single hot loop: `frame = source.get(timeout = T_frame)`; on frame: decode → `imu.state_at(t_mid)` → ego (with `prev_ego` handed to l04's optional compensation) → clutter → tracker → threat → policy → link; on timeout: a health-only tick (RADAR_SILENT evaluation, heartbeat with an advancing `loop_iter` and a stalled `frames_processed`, which the MCU treats as OFFLINE via the health field, not via the stall rule). `sd_notify("WATCHDOG=1")` via the `NOTIFY_SOCKET` datagram (stdlib) from the same loop, rate-limited, only when the loop made progress.
- Per-stage latency and end-to-end (`t_mid` → bytes handed to the writer) with p50/p99/max, logged every 10 s and at exit.
- Recording: JSON-lines (stdlib), one record per raw adapter output and health transition, with a session header (config snapshot, firmware version, RSPI/RRAI, git commit). Writer thread, bounded queue, drop counter. Replay tool feeds l03–l09 from a recording with a `FakeClock` and asserts bit-identical outputs across two runs.
- Deployment doc: `SCHED_FIFO` via `chrt`/`os.sched_setscheduler`, `isolcpus` + `CPUAffinity`, `gc.freeze()` after init with `gc.disable()` on the hot path and a manual `gc.collect()` on timeout ticks, systemd unit with `Type=notify`, `WatchdogSec=2`, `Restart=always`, and `RuntimeWatchdogSec` guidance.

### 4.12 Simulator (`tools/sim/`)
One kinematic truth drives both sensors. The simulator keeps its own `sim_world` coordinates internally for placing vehicles and scatterers; no such frame appears in `goldenfleece/`. Radar model per §13.1 **plus the Doppler-bin model** from datasheet item 6: every scatterer is binned by its relative radial speed (0.78 km/h bins at RSPI 3); scatterers sharing a bin merge into one target with amplitude-weighted (phase-mixed) distance and angle; then the threshold, the ±40° main beam with sidelobe false targets (−12 to −20 dB), 1° angle quantisation, 30 cm range bins, the blind band, the 12-cap (weakest dropped first, tagged unverified), angle noise, false alarms, frame gaps, blockage. IMU model: gyro/accel with bias and noise from the same kinematics including centripetal terms. Ground truth exported per frame.

---

## 5. 12-target-cap contention estimate (analytical; the simulator refines it, the probe measures it)

Because raw targets are Doppler bins, stationary clutter cannot occupy more bins than its Doppler band spans. With the rider at speed `v` and the ±40° main beam, stationary scatterers recede at `v·cos(az) ∈ [0.766 v, v]`, a band of `0.234 · v · 3.6` km/h:

| Rider speed | Clutter Doppler band | Max clutter bins (0.78 km/h) | Free slots for vehicles + false alarms |
|---|---|---|---|
| 3 m/s | 2.5 km/h | 4 | 8 |
| 5 m/s | 4.2 km/h | 6 | 6 |
| 8 m/s | 6.7 km/h | 9 | 3 |
| 10 m/s | 8.4 km/h | 11 | 1 |
| 12 m/s | 10.1 km/h | 13 | **0 (clutter alone exceeds the cap)** |

A car contributes roughly 2–5 bins (body, plus wheel/arch micro-Doppler spread), sidelobes and false alarms 0–2. **Conclusion:** in scatterer-rich surroundings (parked cars, fences, walls) the cap is plausibly hit routinely at ≥ 8 m/s, and above ~11 m/s stationary clutter alone can fill it. Whether a real vehicle is then dropped depends on the sensor's undocumented truncation order (item 10) and on whether it emits every bin above threshold or only local maxima. This is a **§15.3 pre-emptive flag**: the probe's cap-hit rate on a real ride decides it.

Mitigations and costs (§8): `THOF` up — costs detection range for the car (the very target we want); `DEDI = approaching` — removes all stationary clutter and solves contention outright, but makes ego-motion permanently invalid, removes the pacer's receding phase, and weakens blockage detection to "no targets at all while moving"; `MISP` — useless here because stationary clutter sits at 14–18 km/h, above any floor that would still let slow-closing cars through. If the probe shows routine saturation, my recommendation would be `DEDI = approaching` with those costs stated, but that is a team decision, not mine.

---

## 6. Latency budget (estimate) — see `docs/latency_budget.md`

Summary: `t_mid` → bytes at the MCU ≈ 14.5 ms (half frame) + δ_sensor (unknown, placeholder 10 ms) + 1.5 ms wire at 921600 + 1–3 ms USB/thread wake + 2–4 ms Python pipeline + < 1 ms queue + 1–2 ms CDC transfer ≈ **30–45 ms typical, p99 target < 60 ms**. Timestamp skew at 15 m/s: 0.45–0.7 m, inside the §7 budget. The variable, target-count-dependent term is the wire transfer, which is why 921600 is chosen.

---

## 7. ICD outline (`docs/mcu_icd.md`, written in Stage 3)

- **Transport:** USB CDC (device path from config), 8N1 logical, COBS frames delimited by `0x00`, CRC-16/CCITT-FALSE over `[ver][type][seq][payload]`.
- **Pi → MCU:** `WARNING (0x01)` {level u8, side u8, t_arrival_bucket u8, health_state u8, health_bits u16, pipeline_seq u32, t_decided_ms u32}; `HEARTBEAT (0x02)` {loop_iter u32, frames_processed u32, last_frame_number u32, health_state u8, health_bits u16}; `RADAR_CONFIG (0x03)` {rrai, rspi, baud u32, blind_band_cms u16, fallback thresholds}; `HEALTH (0x04)` {state, bits, onset_ms u32}.
- **MCU → Pi:** `STATUS (0x81)` {mode u8 (NORMAL / FALLBACK_OFFLINE), mcu_ms u32, last_seq_applied u32, hb_age_ms u16, outage_count u16}; `OUTAGE_EVENT (0x82)` {start_ms, duration_ms, cause u8}.
- **Heartbeat honesty:** emitted from the pipeline loop only; ≤ 50 ms apart in every loop mode (frame-driven at 29 ms, timeout ticks at 50 ms). **MCU fallback rule:** `loop_iter` not advancing for **250 ms** (5 heartbeats) or no heartbeat for 250 ms ⇒ FALLBACK, render "warnings offline". Worst-case Pi hang → rider told ≈ **0.3 s** (250 ms + render). Justification: a 20 m/s closer covers 6 m in that time; shorter would flicker on ordinary USB/GC hiccups (a CDC re-enumeration takes 1–2 s and *should* trip it).
- **Truth table** (alert line × heartbeat alive / progress-stalled / absent), including stuck line: asserted > `T_line_max = 300 ms` with heartbeat stalled or absent ⇒ **fault**, not threat. Health rendering must be perceptually distinct from threats; a fault is never rendered as ALERT.
- **Physical constraints recorded:** under `PI_POLLS` the K-LD7 falls silent when the Pi stops polling, so MCU fallback means "radar data unavailable → warnings offline"; the MCU must be powered independently of the Pi (or have hold-up) to announce a Pi brownout.
- **Outage logging** on the MCU per event (duration, cause) reported through `OUTAGE_EVENT` and `STATUS`.

---

## 8. Hardware-dependent assumptions — see `docs/UNVERIFIED.md` (initial register)

---

## 9. Questions and flags for the team (answer before or during Stage 1; none blocks Stages 1–4)

1. **Which USB adapter carries the K-LD7?** Two are attached: the PL2303 (`ttyUSB0`) and the MCP2221 (`ttyACM0`). Please confirm the by-id path for `radar.yaml`. If it is the MCP2221, 921600 8E1 is probably not achievable and the config default will be 460800 after the probe confirms.
2. **CPX device path** for `MCU_TRANSPORT = USB_CDC` (`/dev/serial/by-id/...` once the board is plugged in with firmware).
3. **§15.3 pre-emptive flag:** the analytical estimate says cap saturation may be routine at ≥ 8 m/s in clutter (§5). The mitigation choice (`DEDI = approaching` vs `THOF`) is a team decision after the probe measures the real cap-hit rate.
4. **Truncation order when > 12 bins exceed threshold** — not in the datasheet; if the team can ask RFbeam, the answer changes the simulator model's tag from unverified to measured.
5. **Alert-line pin:** GPIO17 proposed (boot pull-down measured here; idle LOW, asserted HIGH, MCU-side pull-down). Which CPX pad receives it?
6. **MCU power independence** (§10.3 precondition): is the CPX powered from the UPS rail directly or through the Pi?
7. **Recording path:** only the root card exists on this Pi. I plan `/home/ted/golden_fleece_recordings/` on it with the stall risk documented, unless a USB drive is provided.
8. **Environment setup consent:** I plan to create `.venv` (with system site packages, so the apt numpy and gpiod are reused) and install scipy, pytest, hypothesis, adafruit-circuitpython-bno08x and adafruit-blinka at Stage 1. SPI0 enablement (`dtparam=spi=on` in `/boot/firmware/config.txt`, reboot) is a system change I would like the team to make or approve at Stage 5.
9. **Design decisions to confirm:** receding movers reach the tracker with a stricter confirmation (§4.6); UNEXPLAINED coasts hold their level rather than escalate (§4.8); heartbeat 50 ms / stall 250 ms / `T_line_max` 300 ms (§7).
10. **Speed aliasing above 27.8 m/s relative** at RSPI 3 (datasheet item 11) is a sensor limit with no software fix; recorded as a limitation unless the team objects.
11. **δ_sensor** has no bench method other than the timing-statistics approach in §4.1; if the team has a better bench (e.g. a chopper wheel with an encoder pulse), it replaces mine.

---

## 10. Stage plan and dependencies

| Stage | Deliverables | Gate |
|---|---|---|
| 1 | venv + deps; `clock.py`, `types.py`, `config.py`, `frames.py`, `health.py`; the three YAML files with the register; invariant tests (grep tests for time/threading/lane words/negations/third-party kld7, compose mismatch, source tag, no-z); simulator skeleton with ground truth export | tests green |
| 2 | l03 → l09 bottom-up, each with unit tests + scenario metrics from the simulator (lead time, misses, false alarms/h, flicker, ego diagnostic numbers, blockage false-alarm rate, cap-hit rate) | per-layer tests green, metrics reported |
| 3 | ICD, COBS/CRC, l10 pure half, MCU emulator with watchdog/fallback/outage log, fault injection, `tools/outage_report.py` | emulator tests incl. stall detection and restart ordering |
| 4 | orchestrator, recording, replay, full-pipeline simulator runs, measured latency → `latency_budget.md` | bit-identical replay, latency p50/p99 |
| 5 | l01 driver + probe, l02 adapter + probe, l10 transport + GPIO, power monitor, systemd unit, bring-up checklist | probes ready; stop if no hardware |
| 6 | hardware bring-up with the team (signs, range scale, six-orientation IMU test) | §15 rules |

Each stage ends with a commit and a short report (passed / placeholder / unverified).

# Latency budget

Status: **Stage 4 — Pi-side rows measured on this Raspberry Pi 5 (simulator-driven, real clock); sensor and USB rows remain estimates until the Stage 6 probe.** Every row carries its source tag.

## Measured (Stage 4, Pi 5 Model B, Python 3.11, venv numpy 2.4 / scipy 1.17)

Full `Runner.step()` for one radar frame = l03 → l09 plus link encode and hand-off (loopback transport into the MCU emulator, so the emulator's parsing is included as overhead; measured before the MCU was removed — since then the hand-off is a swap under a lock and the same overtake runs p50 0.78 / p99 2.03 / max 2.16 ms with `tools/sim/run.py --bench`):

| Scenario | frames | loop p50 | loop p99 | loop max |
|---|---|---|---|---|
| overtake_left_10mps (light clutter) | 293 | 0.77 ms | 1.94 ms | 2.02 ms |
| two_vehicles_crossing | 310 | 1.10 ms | 2.39 ms | 2.76 ms |
| dense_clutter_cap (12 targets every frame) | 276 | 2.11 ms | 2.98 ms | 3.40 ms |

Per-stage p99 over the whole 18-scenario suite (worst scenario): decode 0.06 ms, IMU state 0.6 ms, ego-motion 1.9 ms, clutter 0.14 ms, tracking 1.3 ms, threat 0.07 ms, policy 0.17 ms. These are without `SCHED_FIFO`, `gc.freeze()` or an isolated core, with the desktop session running: the Stage 0 estimate of 2–4 ms typical holds, and rows 5–12 below are replaced by **≤ 3.4 ms max measured**.

## Warning path: radar `t_mid` → motor switched

Configuration assumed: RSPI = 3 (29 ms frame), RRAI = 2, 921600 8E1, `PI_POLLS`, `USB_CDC`.

| # | Stage | Estimate | Tag | Basis / how it is measured |
|---|---|---|---|---|
| 1 | Radar integration, `t_mid` → end of frame | 14.5 ms | nominal | T_frame/2, datasheet typical 29 ms; probe measures T_frame |
| 2 | Sensor processing δ_sensor (FFT, target extraction) → first PDAT byte | **unknown**, placeholder 10 ms | null → measure | Probe: GNFD→PDAT-header delay statistics (minimum if free-running; mean − T_frame if poll-triggered) |
| 3 | Wire transfer PDAT payload + DONE after the timestamped header | 1.3 ms worst (12 targets) | nominal | 108 B × 11 bits / 921600; 10.3 ms at 115200 (the reason for 921600) |
| 4 | USB bridge + kernel + reader-thread wake | 1–3 ms (up to 16 ms if the bridge has an FTDI-style latency timer) | unvalidated | Probe: header-arrival jitter vs DONE cadence |
| 5 | Queue hand-off to the hot loop | < 0.2 ms | estimate | measured per stage |
| 6 | l03 decode | < 0.2 ms | estimate | per-stage timer |
| 7 | l04 state query at `t_mid` (interpolation only; propagation runs in the adapter thread's samples as they arrive) | < 0.3 ms | estimate | per-stage timer |
| 8 | l05 ego-motion RANSAC (≤ 12 points, ≤ 66 minimal sets) | 0.5–1 ms | estimate | per-stage timer |
| 9 | l06 clutter | < 0.1 ms | estimate | per-stage timer |
| 10 | l07 tracking (clustering, ≤ 12×N gate, Hungarian, KF updates) | 0.5–1.5 ms | estimate | per-stage timer |
| 11 | l08 threat + l09 policy | < 0.3 ms | estimate | per-stage timer |
| 12 | l10 hand-over (swap under a lock, wake the render thread) | < 0.05 ms | estimate | per-stage timer |
| 13 | Render-thread wake + PWM duty write | p50 0.04 ms, p99 0.5 ms (no GPIO cost) | measured 2026-10-07 | `hap` records' `lat` (decision → motor write) |
| | **Total, typical** | **≈ 30–45 ms** (Pi-side share measured ≤ 3.4 ms) | | |
| | **Total, p99 target** | **< 60 ms** (GC and scheduling jitter) | | Pi-side rows measured (above); sensor rows Stage 6 |

Python-side budget (rows 5–12): 2–4 ms typical. Scheduling is a small slice, as §1 of the task expects; `SCHED_FIFO` and `gc.freeze()` are opt-in and only trim the p99.

Timestamp-skew consequence (§7): at 15 m/s closing, 30–45 ms of unmodelled latency is 0.45–0.7 m of range error, below one range bin's worth of concern at 30 m and far below the yaw term. The pipeline compensates known latency by predicting tracks to `t_decided`, so only the *unmodelled* part (δ_sensor error, USB jitter) counts.

## Polling overhead

If the sensor acquires continuously, polling costs nothing beyond row 3–4. If acquisition is triggered by GNFD, the frame period becomes T_frame + δ_sensor + wire + host turnaround (~4 ms), i.e. ~35–45 ms instead of 29 ms. The probe's measured period decides which model holds.

## Health onset → haptics (worst case, estimate)

Loop tick under radar silence: 50 ms. Hand-over to the render thread: ≤ 5 ms (its period). Health transitions are handed over as they happen.

| Reason | State | Detection | Worst-case onset → rendered | Tag |
|---|---|---|---|---|
| `RADAR_SILENT` | OFFLINE | no valid DONE for k·T_frame, k = 5 → 145 ms | 145 + 50 + 5 ≈ **200 ms** | unvalidated |
| `RADAR_GAPS` | DEGRADED | gap fraction > 20 % over a 1 s window | ≈ **1.06 s** | unvalidated |
| `RADAR_POSSIBLY_BLOCKED` | DEGRADED (advisory) | in motion, no targets and no inliers for 5 s | ≈ **5.1 s** | unvalidated |
| `PDAT_SATURATED` | DEGRADED | ≥ 50 % of frames at cap over 1 s | ≈ **1.06 s** | unvalidated |
| `IMU_FAULT` | DEGRADED | no sample for 100 ms, or library reset/error | ≈ **160 ms** | unvalidated |
| `EGO_INVALID` | informational | per frame | ≈ 35 ms | — |
| `UNDERVOLTAGE` / `THROTTLED` | DEGRADED | `vcgencmd get_throttled` polled at 1 Hz | ≈ **1.06 s** | unvalidated |
| `HAPTICS_FAULT` | OFFLINE | a motor pin cannot be claimed or written | at once (phone page only: there is nothing to render it on) | — |
| `PIPELINE_RESTARTING` | OFFLINE | handed over before any decision after process start | systemd `RestartSec` (1 s) + interpreter start (~0.5 s); the haptics were already rendering "warnings offline" from the stall (≤ 250 ms after it) | unvalidated |
| Pipeline loop stall (no heartbeat progress) | haptics FALLBACK | render thread: `loop_iter` stalled or absent 250 ms | **≈ 0.26 s** to "warnings offline" | `docs/haptics.md` §4 |
| Whole process hung, Pi brownout | — | nothing independent of the Pi is left | silence or a stuck motor until systemd's watchdog (≈ 2 s) and `ExecStopPost`; brownout: silence | limitations H5, H6 |

Each row became a test in Stage 3/4: the fault is injected in the simulator harness and the time it is rendered is asserted against the column above.

**Measured in simulation (Stage 2/4, FakeClock, 29 ms frames, 50 ms ticks):** RADAR_SILENT onset → detection 0.17 s; RADAR_GAPS 0.25 s (30 % gap probability); IMU_FAULT 0.10 s; RADAR_POSSIBLY_BLOCKED 5.02 s; loop-stall proxy → haptics FALLBACK ≤ 0.27 s (`tests/test_runner_integration.py`); heartbeat with stalled progress counter → FALLBACK ≤ 0.26 s (`tests/test_l10_haptics.py`).

**Open-road blockage advisory false-alarm rate (simulator, §10.2):** with no roadside clutter at all and a 0.02/frame noise false-alarm rate, the advisory was active for 282 of 690 frames (41 %) of a 20 s ride. Proposal: severity *advisory* (a quiet marker, never a threat pattern), `min_silent_s` 5 s (measured detection latency 5.0 s when actually blocked), and a rider-facing meaning of "radar sees nothing — check the cover". Real-road clutter density decides the field rate; `tools/outage_report.py` measures it from recordings.

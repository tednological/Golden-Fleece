# Stage reports (what passed, what is placeholder, what is unverified)

Test suite at the end of Stage 5: **120 tests passing** (`.venv/bin/python -m pytest -q`).

## Stage 1 — foundations (commit c7bb759)
**Passed:** Clock (monotonic + fake), typed contracts, quaternion/transform algebra (`compose` raises on frame mismatch; property-tested round trips), config loader with the §12.3 register (`allow_unmeasured` gate, `source` tags mandatory, `world/odom/bike` frames rejected), health aggregation, grep/structural invariant tests, simulator core reproducing every sign convention (car on the rider's left ⇒ positive wire angle ⇒ negative y; approaching ⇒ negative raw speed; +9.81 on +Z at rest; left shoulder check ⇒ positive gyro z; coordinated-turn degeneracy).
**Placeholder:** `sensor_delay_s` null (dev placeholder 10 ms), blind band 0.5 m/s, all thresholds tagged `unvalidated`.
**Unverified:** IMU library units (identity SI conversion), cap truncation order.

## Stage 2 — pure layers l03–l09 (commit 970728b, refined through Stage 5)
**Passed:** golden vectors (§6.3, §9.5) verbatim; ESKF static / tilt / constant-rate / bias / gating tests; RANSAC with outliers, heading diagnostic, merged-bin detection, prior gate; clutter invariant (approaching passes with ego invalid); tracker confirm / every coast reason / crossing tracks / gyro compensation / re-acquisition; threat semantics (`t_arrival`, widening CENTER band, coasting contributes); policy (immediate escalation, dwell + hysteresis, min on-time, BOTH, alert never for a fault, flicker); blockage detector.

Scenario suite (18 scenarios, `tools/sim/run.py`), final numbers:

| Scenario | lead time at first WARNING (truth t_arrival) | missed | false alarms/h | flicker | cap-hit rate | ego valid |
|---|---|---|---|---|---|---|
| overtake left / right 5 m/s | 3.1 s / 3.2 s | 0 | 0 | 0 | 0 | 0.33 / 0.38 |
| overtake left / right 10 m/s | 2.8 s / 2.7 s | 0 | 0 | 0 | 0 | 0.09 / 0.08 |
| overtake left / right 20 m/s | 1.2 s / 1.3 s | 0 | 0 | 0 | 0 | 0.01 / 0.03 |
| close pass 1 m (FOV_EXIT) | 3.2 s | 0 | 0 | 0 | 0 | 0.22 |
| same-speed pacer through the blind band | 3.3 s | 0 | 0 | 0 | 0 | 0.27 |
| two vehicles, crossing azimuths | 2.0 s / 3.9 s | 0 | 0 | 0 | 0.06 | 0.36 |
| overtake during 50° / −40° shoulder checks | 2.8 s | 0 | 0 | 0 | 0 | 0.30 |
| 20° cornering lean | 2.8 s | 0 | 0 | 0 | 0 | 0.08 |
| open road, no clutter | 2.3 s | 0 | 0 | 0 | 0 | 0.00 (by design) |
| dense clutter (cap) | 2.1 s | 0 | 0 | 0 | **0.68** | 0.99 |
| stopped at a light | 3.2 s | 0 | 0 | 0 | 0 | 1.00 (IMU stopped) |
| radar blocked mid-ride | missed by design; RADAR_POSSIBLY_BLOCKED after 5.02 s | 1 | 0 | 0 | 0 | 0.12 |
| radar frame gaps (30 %) | 2.7 s; RADAR_GAPS after 0.25 s | 0 | 0 | 0 | 0.03 | 0.18 |
| radar silence 3 s | 2.8 s; RADAR_SILENT after 0.17 s | 0 | 0 | 0 | 0 | 0.01 |
| IMU fault | 2.7 s; IMU_FAULT after 0.10 s | 0 | 0 | 0 | 0 | 0.18 |

Lead times are governed by the *unvalidated* 3 s WARNING threshold (`t_arrival_low < 3 s`), so ≈ 3 s at low closing speed and less when the car is first detected inside 3 s of arrival (20 m/s: detection at ~32 m gives 1.2 s — a physical limit of a 30 m radar, not a tuning issue; ADVISORY fires earlier).

**Ego-motion heading diagnostic (§9.5, §15.10 — reported, not wired in):** when ego-motion is valid, `psi_travel` error vs the true torso yaw has mean ≤ 1.2° and std 1.2–2.9° in every clutter scenario (5.6° with two vehicles); it validates well on synthetic data. Validity itself is clutter-limited (0.08–0.38 at the suburban 0.15 scatterers/m used; 0.99 in dense clutter).

**12-target cap (§9.6, §15.3):** dense clutter at 9 m/s hits the cap in 68 % of frames and the overtake is still warned (2.1 s lead) with the simulator's `weakest_first` truncation; with `fft_order` truncation the outcome depends on the sensor's real policy (unverified R8). The analytical estimate in the Stage 0 plan stands; the probe measures the real rate.

**Blockage advisory false alarms (§10.2):** open road with no clutter: advisory active 41 % of the ride (282/690 frames). Proposal: advisory severity, 5 s window, meaning "radar sees nothing — check the cover".

**Three sensor artefacts found by the simulator and handled with physical signatures** (all configurable, tagged unvalidated): range aliasing beyond the 30 m setting (l06 magnitude-vs-range check), wheel micro-Doppler phantoms (l07 range-trend vs range-rate consistency; l06 static-range phantom filter), coasting tracks re-acquiring random false alarms (absolute re-acquisition gates).

**Placeholder:** every threshold; the simulator's magnitude scale, false-alarm rate and micro-Doppler model.

## Stage 3 — link and health
**Passed:** ASCII line protocol with CRC-16/CCITT-FALSE (check value 0x29B1 verified), round trip and corruption rejection, resync; MCU emulator: progress-stall detection with bytes still flowing (≤ 0.31 s), absent heartbeat (≤ 0.26 s), alert staleness (300 ms) and alert dropped in fallback, fault never rendered as threat, DEGRADED keeps the threat, restart announced before any threat, outage log + STATUS; recording format round trip; outage report (episodes, durations, causes, longest, riding time).
**Deliverables:** `docs/mcu_icd.md`, `goldenfleece/l10_mcu_link/{protocol,emulator}.py`, `tools/outage_report.py`.
**Design change from the plan (team answer 5):** keyword ASCII lines instead of COBS binary; the UART carries the alert (`ALERT` keyword) instead of a GPIO line.

## Stage 4 — orchestrator, recording, replay, latency
**Passed:** runner integration on the simulator with the real link and emulator: overtake warned with restart announced first; Pi-hang proxy → FALLBACK ≤ 0.27 s ("warnings offline" under PI_POLLS); stalled progress with heartbeats flowing → FALLBACK; link loss reported and recovered; alert precedes the WARN of the same decision; undervoltage flag reaches the MCU within 1 s; recordings replay bit-identically and match the recorded decisions (0 mismatches over 293 frames).
**Measured (this Pi 5, real clock):** full `Runner.step()` per frame p50 0.8–2.1 ms, p99 1.9–2.9 ms, max 3.1–3.4 ms; per-stage p99: ego 1.9 ms, tracking 1.3 ms, IMU state 0.5 ms, others < 0.2 ms. Health onset → detection (simulation): silence 0.17 s, gaps 0.25 s, IMU 0.10 s, blockage 5.02 s; all inside the documented bounds (`tests/test_health_onset.py`). `docs/latency_budget.md` updated.
**Deployment (documented, opt-in):** systemd unit with `Type=notify`/`WatchdogSec=2`, `WATCHDOG=1` from the loop on progress; `--deploy` for SCHED_FIFO + gc.freeze; isolcpus guidance; telemetry snapshot.

## Stage 5 — adapters and probes (no hardware attached to this build)
**Passed:** K-LD7 protocol against the datasheet's own vectors (Fig. 17/18, Table 15; Fig. 19 erratum documented), RPST pack/unpack, parser resync and fuzzing; driver state machine against a simulated sensor: stale-baud recovery (GBYE probe → INIT → GRPS → SRPS with the version string → GRPS verify), refusal to stream on parameter mismatch (RESP 3), frame timestamps, gap detection, silence → RADAR_SILENT → reconnect, resync on garbage, threaded smoke test; `tools/kld7_probe.py --fake` dry run (29 ms period, 0 gaps, achieved baud reported).
**Written, untested on hardware:** `l02_imu_data_input/bno085_spi.py` (the installed library accepts `report_interval`; new samples detected by reading identity), `tools/bno085_probe.py` with the §9.2 stop rule, `tools/run_pipeline.py`, `deploy/goldenfleece.service`, `docs/deployment.md`, `docs/bringup_checklist.md`.
**Environment facts for Stage 6:** SPI0 is not enabled on this Pi (`dtparam=spi=on` needed); the radar bridge is the PL2303 at `/dev/ttyUSB0`; the MCU is the ATmega328P-XMINI at `/dev/ttyACM1`; the MCU is powered through the Pi (brownout cannot be announced: documented limitation).

## Stage 6 — pending (hardware, with the team)
Follow `docs/bringup_checklist.md`. Stop-and-ask on any sign disagreement; never add a compensating negation.

## Stage 6 change — MCU removed (2026-10-07)
**Decision (team decision 12):** the MCU is gone; the PWM vibration motors are mounted on the Pi's GPIOs and driven at 100 % duty.
**Replaced:** `l10_mcu_link` (line protocol, writer thread, MCU emulator), `tools/mcu_emulator_serial.py`, `tools/vest_display.py` and `docs/mcu_icd.md` by `goldenfleece/l10_haptics/` (pure patterns and rendering priority, a PWM motor adapter, a render thread with the stall watchdog), `tools/haptics_test.py` and `docs/haptics.md`. `MCU_LINK_DOWN` became `HAPTICS_FAULT` (OFFLINE); `uart` records became `hap` records; `run_pipeline.py --no-link/--link-port` became `--no-haptics`.
**Passed:** pattern timing; the configured patterns meet the rendering requirements (fault pulses shorter than threat pulses, rising on-time, alert the only maximum) and unsafe ones are refused at load; rendering priority (fault never a threat, DEGRADED keeps the threat); side mapping; loop stall → FALLBACK ≤ 0.27 s on the simulator, stalled counter with heartbeats flowing → FALLBACK; restart rendered "offline" before any threat; a lost motor → HAPTICS_FAULT and recovery; the bench tool against fake motors. Decision → motor write measured p50 0.04 ms, p99 0.5 ms (render thread, no GPIO).
**Not yet done:** on the hardware: `tools/haptics_test.py` with the real motors (pins, side, 100 % level), patterns on the body while riding (`haptic_patterns`), `kill -STOP` / `kill -9` of the service with a motor on (H5).

# UNVERIFIED — assumptions only hardware can confirm

Initial register (Stage 0). Each item names the confirming step. Items move out of this file only with a measured value and a date.

## Radar (K-LD7)

| # | Assumption | Source of the assumption | Confirmed by |
|---|---|---|---|
| R1 | Positive `angle_raw` = target on the sensor's right = **rider's LEFT** (rear-facing) = **negative y** | Datasheet text (angle flag 1 = Right for angle > 0°) and Fig. 5 reading | Stage 6 moving reflector on the rider's left ⇒ positive raw angle. If not, **stop and ask** (§15.1); never negate. |
| R2 | Negative raw speed = approaching | Datasheet p.6 | Stage 6 reflector approaching |
| R3 | Range scale 1 cm per count, 30 cm bins at RRAI 2 | Datasheet Table 3/13 | Stage 6 tape measure |
| R4 | Frame duration per RSPI = 229 / 114 / 57 / 29 ms | Datasheet typical | **Measured 2026-09-14: 228.3 / 114.2 / 57.0 / 28.76 ms (p50), 0 gaps at every setting.** In config, tagged measured |
| R5 | δ_sensor (end of integration → first PDAT byte) | unknown | **Measured 2026-09-14 (`--random-phase`, 672 frames): 8.94 / 8.24 / 8.97 ms from min / mean − T/2 / max − T; config `sensor_delay_s: 0.0087`** |
| R6 | Acquisition is free-running and GNFD selects the next completed frame (vs poll-triggered) | Datasheet Fig. 14 wording | **Confirmed 2026-09-14: random-phase poll→header delay spans exactly one frame period (28.7 ms spread), i.e. δ + U(0, T)** |
| R7 | PDAT never exceeds 12 targets | 96-byte max payload | Static bench 2026-09-14: max 6 targets/frame, mean 0.11, cap-hit 0 (a ride is still needed for R10) |
| R8 | When more than 12 bins exceed threshold, the sensor keeps the strongest | none (simulator convention) | RFbeam / probe with a controlled scene; **unknown until then** |
| R9 | Every FFT bin above threshold becomes a raw target (vs local maxima only) | Datasheet Fig. 4 wording | Probe: count of targets vs rider speed in clutter |
| R10 | Cap-hit rate on real rides | analytical estimate says routine at ≥ 8 m/s in clutter | Probe on a ride; §15.3 decision |
| R11 | Doppler blind band width at RSPI 3 (placeholder 0.5 m/s, `unvalidated`) | one bin = 0.217 m/s, DC bin removed | Bench reflector at controlled speeds |
| R12 | Sensor firmware version string | unknown | **Measured 2026-09-14: `K-LD7_APP-RFB-0105`** (datasheet placeholder pattern `K-LD7_APP-RFB-XXXX`) |
| R13 | 921600 8E1 achievable end-to-end (bridge + sensor) | bridge dependent | **Confirmed 2026-09-14 on the PL2303: connected first try at 921600, 701 RESP OK, 0 UART errors, 0 resync bytes** |
| R14 | Baud recovery: GBYE at each rate yields RESP within 100 ms | protocol reading | Probe after a deliberate kill without GBYE |
| R15 | Early GNFD (before the previous frame's data arrives) is either queued or answered RESP 5 | unknown | Probe experiment |
| R16 | Vest fabric over the radar acts as a radome without false targets from flapping | datasheet radome warning says it may not | Ride test; blockage heuristic false-alarm rate |
| R17 | RESP 4 (UART error) rate at the chosen baud over the actual cabling | none | **0 in 701 responses over 20 s at 921600 (2026-09-14)** |
| R18 | Header-arrival timestamp jitter through the USB bridge < 3 ms p99 | typical | **Confirmed: 1.86 ms p99 (2026-09-14)** |
| R19 | Relative speeds above 27.8 m/s alias (wrong sign possible) | datasheet p.9 | Accepted limitation (team answer 10); no test planned |
| R20 | Wheel micro-Doppler (wheel-top returns at ~2x vehicle speed, contact patches at road speed) appears in PDAT as modelled in the simulator | automotive-radar experience, not the datasheet | Rides: `tracker.consistency_*` and `clutter.phantom_filter` are harmless if absent |
| R21 | Range aliasing: a strong car beyond the RRAI range wraps to a short apparent range with the right Doppler | datasheet p.9 "false reflections" | Probe magnitude-vs-range log with a reflector; recalibrate `clutter.alias_check.c_alias_db` (sim value 63 dB assumes the sim's magnitude scale). **Check disabled 2026-09-14 (see R22): range-aliased returns are unfiltered until it is recalibrated** |
| R22 | The sensor's magnitude scale (dB x100) is comparable to the simulator's (10 log RCS - 40 log r + 80) | assumption for the alias check only | **Contradicted at short range 2026-09-14 (bench recording, 23,049 frames): approaching returns within 5 m have median 52 dB, so c_alias 63 rejected 81 % of them; replay gave 0 warnings with the check on, 7 episodes (up to ALERT) with it off.** Probe with a reflector to recalibrate |

## IMU (BNO085)

| # | Assumption | Confirmed by |
|---|---|---|
| I1 | The mandated library can deliver ≥ 100 Hz gyro over SPI on the Pi 5 with usable timing (jitter p99 < 5 ms) | Stage 5 measurement; **stop and report** if not (§15.4) |
| I2 | The library returns rad/s and m/s² (identity SI conversion) | +9.81 on +Z static test |
| I3 | `T_radar_imu` is a signed permutation (axis-aligned mount) | Six-orientation static test (Stage 6) |
| I4 | Accelerometer bias small enough to skip estimating (< 0.05 m/s²) | Six-orientation test residuals |
| I5 | Gyro bias stability adequate for Δψ over track lifetimes (< 0.5°/s drift) | Static log |
| I6 | INT-edge timestamping is available through the library | Stage 5 (falls back to read-time stamps) |
| I7 | SPI0 enabled (`dtparam=spi=on`) and Blinka works on this Pi 5 kernel | **2026-09-14: `/dev/spidev0.0` present; Blinka imports (`RASPBERRY_PI_5`) and claims GPIO5/24/25.** SPI traffic to the sensor not yet tested |
| I8 | Wiring CS=D5 (GPIO5, header pin 29), INT=D25, RESET=D24 as in `pipeline.yaml: imu.pins`. Not CE0: with `dtparam=spi=on` the kernel owns GPIO8 as `spi0 CS0`, and claiming it fails with `GPIO busy` (reproduced 2026-09-14) | assumed | Bring-up |
| I9 | The library's `_readings` identity change is a reliable new-sample detector (no per-sample timestamps in the library) | code reading of adafruit_bno08x 1.x | `tools/bno085_probe.py` rate check |

## MCU link and GPIO

| # | Assumption | Confirmed by |
|---|---|---|
| M1 | GPIO17 boot-default pull-down on this Pi 5 (measured via `pinctrl`: yes) also holds during reboot/firmware stages | Scope or meter during a reboot |
| M2 | MCU-side pull-down defines idle; an unpowered Pi reads idle | Bench |
| M3 | CPX USB CDC enumerates with a stable by-id path and survives Pi USB re-enumeration | Stage 5 |
| M4 | STATUS round trip < 10 ms (latency measurement resolution) | Stage 5 |
| M5 | **Not met (team answer 6):** the MCU is powered through the Pi, so a Pi brownout silences both and cannot be announced. Documented limitation (ICD §9.2). | Team hardware decision |
| M7 | The alert travels on the UART (team answer 5): its failure domain is the serial link's; a stuck assertion is bounded by the 300 ms stale rule | ICD §5 | Stage 5 bench with `kill -STOP` |
| M8 | ATmega328P-XMINI mEDBG CDC bridge sustains 57600 8N1 with ≤ 96-byte lines at up to ~40 lines/s | assumption | Stage 5 bench (`STAT` round trips, CRC error count) |
| M6 | Under `PI_POLLS`, a passive tap of radar TX carries nothing during a Pi hang | By construction; documented in ICD |

## Power and platform

| # | Assumption | Confirmed by |
|---|---|---|
| P1 | `vcgencmd get_throttled` bits reflect UPS brownouts on the 5 V rail | Bench with a load |
| P2 | SD-card write stalls do not stall the hot loop (writer thread + bounded queue) | Stage 4 latency measurement while recording |
| P3 | systemd `WatchdogSec` restart path announces `PIPELINE_RESTARTING` within the documented time | Stage 4 harness + Stage 5 on-device |

## Thresholds tagged `unvalidated` (simulator-tuned, then rides)

Threat time thresholds, `r_close`, side band constants, policy dwell / hysteresis / min on-time, blockage timeout, gap-fraction threshold, cap-saturation threshold, RANSAC thresholds, ESKF gates and noise densities, coast σ bounds, cluster distances.

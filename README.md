# Golden Fleece — rear radar + IMU warning pipeline (Raspberry Pi 5 side)

A wearable cyclist vest warns the rider of vehicles overtaking from behind. A rear-facing RFbeam K-LD7
24 GHz Doppler radar and a BNO085 IMU on one rigid back plate feed this pipeline; the Pi does soft
real-time estimation and drives PWM vibration motors on its own GPIOs (one per side, 100 % duty while on; no MCU)
with patterns for the threat level, the side and "warnings offline". No computer vision. No lane semantics: the system can say "something is closing
fast from behind, roughly left or right", nothing more. A USB camera records footage for reviewing rides into
`Camera Footage/`; nothing in the pipeline reads it.

```
+X -> behind the rider      +Y -> rider's RIGHT      -Y -> rider's LEFT      +Z -> up
```
A car on the rider's LEFT gives a POSITIVE K-LD7 angle and lands at NEGATIVE y. The only conversion site
is `goldenfleece/l03_radar_decode/decode.py`.

## Layout
```
goldenfleece/   clock.py types.py config.py frames.py health.py
                l01_radar_data_input  (K-LD7 driver, from the datasheet)   l02_imu_data_input (BNO085 SPI adapter)
                l03_radar_decode      l04_imu_decode (ESKF)   l05_ego_motion (RANSAC)   l06_clutter_rejection
                l07_tracking (r/r_dot + azimuth KFs, Hungarian, coast reasons)   l08_threat   l09_warning_policy
                l10_haptics (patterns, PWM motors, render thread + stall watchdog)   orchestrator (pipeline, runner, recording, latency)
tools/          sim/ (synthetic world, scenarios, metrics, runner bench)  kld7_probe.py  bno085_probe.py  replay.py
                outage_report.py  run_pipeline.py  web_app.py (field web app)  haptics_test.py (motor bench test)
                camera_recorder.py (USB camera -> Camera Footage/)  fake_kld7.py  fake_bno085.py
config/         radar.yaml  frames.yaml  pipeline.yaml (thresholds + the unmeasured-parameters register)  camera.yaml
docs/           STAGE0_PLAN.md  STAGE_REPORTS.md  haptics.md  latency_budget.md  UNVERIFIED.md  DECISIONS.md
                deployment.md  bringup_checklist.md  field_testing.md
deploy/         goldenfleece.service (pipeline)  goldenfleece-web.service (field web app)
                goldenfleece-camera.service (camera recorder)
Camera Footage/ camera recordings (git ignores it)
tests/          unit / property / invariant / integration tests
```

## Quick start
```bash
python3 -m venv --system-site-packages .venv && .venv/bin/pip install scipy pytest hypothesis adafruit-circuitpython-bno08x adafruit-blinka
.venv/bin/python -m pytest -q                                   # all tests
.venv/bin/python -m tools.sim.run                               # 18-scenario suite with metrics
.venv/bin/python -m tools.sim.run overtake_left_10mps --verbose --record-dir /tmp/rec
.venv/bin/python tools/replay.py /tmp/rec/overtake_left_10mps.jsonl --twice
.venv/bin/python tools/outage_report.py /tmp/rec/*.jsonl
.venv/bin/python -m tools.sim.run --bench                       # real Runner loop timing on this Pi
.venv/bin/python tools/kld7_probe.py --seconds 20 --all-rspi    # hardware: see docs/bringup_checklist.md
.venv/bin/python tools/bno085_probe.py --seconds 30
.venv/bin/python tools/run_pipeline.py --config config           # the pipeline (see docs/deployment.md)
.venv/bin/python tools/web_app.py                               # field web app on :8080 (see docs/field_testing.md)
.venv/bin/python tools/camera_recorder.py --status              # camera recorder: what it is doing (--off / --on)
.venv/bin/python tools/haptics_test.py --patterns               # motors: pins, sides, 100 % duty, patterns (service stopped)
.venv/bin/python tools/run_pipeline.py --no-haptics              # no GPIO: HAPTICS log lines show what would be felt
```

## Rules the code enforces (tests in `tests/test_invariants.py` and friends)
* hardware-touching code contains no math; all time flows through `Clock`; threads only in adapters;
* l03–l09 are pure and deterministic (recordings replay bit-identically);
* every stage counts in / out / rejected-by-reason; every failure is announced within a documented time;
* no track dies because measurements stopped; no threat resolves by disappearance; the alert channel is
  never asserted for a fault; heartbeats and `WATCHDOG=1` come from the pipeline loop only.

## Not a safety device
This is an unfinished prototype, published so the work can be read and reused. It has not been through the
field testing in `docs/field_testing.md`, and open items are listed in `docs/UNVERIFIED.md`. It cannot see
anything that is not moving relative to the rider, it has no lane semantics, and a silent system never means
the road is clear. Do not rely on it to decide whether it is safe to move, and keep riding as though it were
not there. No warranty: see LICENSE.

## Status
Stages 0–5 complete. Stage 6 (hardware bring-up) in progress: K-LD7 timing measured, the BNO085 running on
l02's own SPI transport (gyro ~200 Hz), the field web app, the camera recorder and their systemd services in place. Since
2026-10-07 there is no MCU: the vibration motors hang off the Pi (`docs/haptics.md`); check them with `tools/haptics_test.py`. Before riding,
work through `docs/field_testing.md`; open items are in `docs/UNVERIFIED.md`.

## Licence
MIT, see [LICENSE](LICENSE). The copyright line names the GitHub account; change it if you want your legal
name there instead.

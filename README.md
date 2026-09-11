# Golden Fleece — rear radar + IMU warning pipeline (Raspberry Pi 5 side)

A wearable cyclist vest warns the rider of vehicles overtaking from behind. A rear-facing RFbeam K-LD7
24 GHz Doppler radar and a BNO085 IMU on one rigid back plate feed this pipeline; the Pi does soft
real-time estimation and sends warning commands over a serial link to an MCU that owns the hard
real-time haptics/lights. No computer vision. No lane semantics: the system can say "something is closing
fast from behind, roughly left or right", nothing more.

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
                l10_mcu_link (line protocol, writer thread, MCU emulator)   orchestrator (pipeline, runner, recording, latency)
tools/          sim/ (synthetic world, scenarios, metrics, runner bench)  kld7_probe.py  bno085_probe.py  replay.py
                outage_report.py  run_pipeline.py  mcu_emulator_serial.py  fake_kld7.py
config/         radar.yaml  frames.yaml  pipeline.yaml (thresholds + the unmeasured-parameters register)
docs/           STAGE0_PLAN.md  STAGE_REPORTS.md  mcu_icd.md  latency_budget.md  UNVERIFIED.md  DECISIONS.md
                deployment.md  bringup_checklist.md
deploy/         goldenfleece.service
tests/          120 unit / property / invariant / integration tests
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
```

## Rules the code enforces (tests in `tests/test_invariants.py` and friends)
* hardware-touching code contains no math; all time flows through `Clock`; threads only in adapters;
* l03–l09 are pure and deterministic (recordings replay bit-identically);
* every stage counts in / out / rejected-by-reason; every failure is announced within a documented time;
* no track dies because measurements stopped; no threat resolves by disappearance; the alert channel is
  never asserted for a fault; heartbeats and `WATCHDOG=1` come from the pipeline loop only.

## Status
Stages 0–5 complete and committed; Stage 6 (hardware bring-up) is pending: see `docs/STAGE_REPORTS.md`,
`docs/bringup_checklist.md` and `docs/UNVERIFIED.md`.

# Deployment (opt-in; correctness never depends on any of this)

## Run manually
```bash
.venv/bin/python tools/run_pipeline.py --config config
```
Options: `--no-imu` (radar only, IMU_FAULT flagged), `--radar sim:<scenario>` and `--imu sim` (real runner and real haptics on the simulator, paced in real time), `--no-haptics` (drive no GPIO; `HAPTICS` log lines and `hap` records show what would be felt), `--deploy` (SCHED_FIFO + gc.freeze, see below), `--log-level DEBUG`.

## systemd unit
`deploy/goldenfleece.service` — `Type=notify`, `WatchdogSec=2`, `Restart=always`, `RestartSec=1`, and `ExecStopPost=tools/haptics_test.py --off`, which switches every vibration motor off whenever the pipeline exits (a killed process can leave a pin high, `docs/haptics.md` §4).
```bash
sudo cp deploy/goldenfleece.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable --now goldenfleece
journalctl -u goldenfleece -f
```
`WATCHDOG=1` is sent **from the pipeline loop on progress** (`goldenfleece/orchestrator/runner.py`, `_watchdog`), rate-limited to 0.5 s; a stalled loop is restarted by systemd within 2 s, and the restarted process hands `PIPELINE_RESTARTING` to the haptics before any decision (they were already rendering "warnings offline" from the stall, ≤ 0.25 s after it).

## Camera recorder
`deploy/goldenfleece-camera.service` runs `tools/camera_recorder.py` (settings in `config/camera.yaml`), a separate
process at `Nice=10` like the web app; the pipeline never reads the camera.
```bash
sudo cp deploy/goldenfleece-camera.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable --now goldenfleece-camera
sudo systemctl restart goldenfleece-web      # the page's camera card and button come with this version of web_app.py
journalctl -u goldenfleece-camera -f
```
Recording is on at every boot; the web app's button (or `tools/camera_recorder.py --off` / `--on`) writes
`/tmp/goldenfleece_camera/control.json`, which the recorder applies within 0.5 s, and the page reads the recorder's
`status.json` beside it. See "Camera" in `docs/field_testing.md`.

## Pi hardware watchdog (documented, not required)
In `/etc/systemd/system.conf` set `RuntimeWatchdogSec=15` and reboot; systemd then pets the BCM watchdog and a kernel hang reboots the Pi. With no MCU this is the only thing that acts on a kernel hang: the Pi reboots, the motor pins come up undriven (off, given the drivers' pull-downs) and the pipeline starts again.

## Scheduling (opt-in)
* `--deploy` requests `SCHED_FIFO` priority 30 for the main thread via `os.sched_setscheduler` (needs `CAP_SYS_NICE`, granted by the unit's `AmbientCapabilities`) and calls `gc.freeze()` after initialisation; the hot loop then runs with the automatic collector disabled and collects explicitly on health-only ticks (no radar frame pending).
* Dedicated core: add `isolcpus=3 nohz_full=3 rcu_nocbs=3` to `/boot/firmware/cmdline.txt`, then `CPUAffinity=3` in the unit. Measure before and after with the `lat` records in the recording (`p99`, `max`); the Stage 4 measurement without any of this is p99 < 3 ms, max 3.4 ms, so this trims tails only.

## Recording
`config/pipeline.yaml: recording.dir` (team decision: root card, `/home/ted/golden_fleece_recordings`). The writer thread has a 4096-record bounded queue and never blocks the loop; drops are counted in the `end` record and reported by `tools/outage_report.py`. SD write stalls therefore cost records, not warnings. Rotate old sessions manually; ~120 MB/h at 200 Hz IMU + 34 Hz radar, plus a few hundred kB/h of `hap` records (one per change in what the motors render, and one every 5 s without a change).

## Before a ride
* `allow_unmeasured: false` in `config/pipeline.yaml` once `sensor_delay_s` is measured; otherwise every start logs the placeholder loudly.
* `radar.yaml: port` is a `/dev/serial/by-id/...` path (stable across re-enumeration).
* Add the user to `dialout` and `gpio`; enable SPI (`dtparam=spi=on`) for the BNO085.
* `tools/haptics_test.py` passes with the service stopped (pins, sides, 100 % level), `docs/haptics.md` §2.

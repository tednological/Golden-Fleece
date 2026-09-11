# Deployment (opt-in; correctness never depends on any of this)

## Run manually
```bash
.venv/bin/python tools/run_pipeline.py --config config
```
Options: `--no-imu` (radar only, IMU_FAULT flagged), `--radar sim:<scenario>` and `--imu sim` (real runner and real MCU link on the simulator, paced in real time), `--no-link` (log decisions only), `--deploy` (SCHED_FIFO + gc.freeze, see below), `--log-level DEBUG`.

## systemd unit
`deploy/goldenfleece.service` — `Type=notify`, `WatchdogSec=2`, `Restart=always`, `RestartSec=1`.
```bash
sudo cp deploy/goldenfleece.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable --now goldenfleece
journalctl -u goldenfleece -f
```
`WATCHDOG=1` is sent **from the pipeline loop on progress** (`goldenfleece/orchestrator/runner.py`, `_watchdog`), rate-limited to 0.5 s; a stalled loop is restarted by systemd within 2 s, and the restarted process announces `PIPELINE_RESTARTING` to the MCU before anything else (the MCU is already in FALLBACK from the heartbeat stall, ≤ 0.25 s after the hang).

## Pi hardware watchdog (documented, not required)
In `/etc/systemd/system.conf` set `RuntimeWatchdogSec=15` and reboot; systemd then pets the BCM watchdog and a kernel hang reboots the Pi. This does not help the rider directly (the MCU already reports "warnings offline"); it shortens the outage.

## Scheduling (opt-in)
* `--deploy` requests `SCHED_FIFO` priority 30 for the main thread via `os.sched_setscheduler` (needs `CAP_SYS_NICE`, granted by the unit's `AmbientCapabilities`) and calls `gc.freeze()` after initialisation; the hot loop then runs with the automatic collector disabled and collects explicitly on health-only ticks (no radar frame pending).
* Dedicated core: add `isolcpus=3 nohz_full=3 rcu_nocbs=3` to `/boot/firmware/cmdline.txt`, then `CPUAffinity=3` in the unit. Measure before and after with the `lat` records in the recording (`p99`, `max`); the Stage 4 measurement without any of this is p99 < 3 ms, max 3.4 ms, so this trims tails only.

## Recording
`config/pipeline.yaml: recording.dir` (team decision: root card, `/home/ted/golden_fleece_recordings`). The writer thread has a 4096-record bounded queue and never blocks the loop; drops are counted in the `end` record and reported by `tools/outage_report.py`. SD write stalls therefore cost records, not warnings. Rotate old sessions manually; ~120 MB/h at 200 Hz IMU + 34 Hz radar.

## Before a ride
* `allow_unmeasured: false` in `config/pipeline.yaml` once `sensor_delay_s` is measured; otherwise every start logs the placeholder loudly.
* `radar.yaml: port` and `pipeline.yaml: link.port` are `/dev/serial/by-id/...` paths (stable across re-enumeration).
* Add the user to `dialout`; enable SPI (`dtparam=spi=on`) for the BNO085.

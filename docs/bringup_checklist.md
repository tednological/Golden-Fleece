# Hardware bring-up checklist (Stage 5 → Stage 6)

Stop-and-ask rules (task §15) apply to every measured sign. Never patch a sign in software.

## A. Radar (K-LD7 on the PL2303 bridge, `/dev/ttyUSB0`)
1. `ls -l /dev/serial/by-id/` — confirm the Prolific path in `config/radar.yaml`.
2. `.venv/bin/python tools/kld7_probe.py --seconds 20` (115200 first, then the configured 921600). Record in `docs/UNVERIFIED.md`:
   * firmware version string (R12); achieved baud (R13); RESP 4 count at that baud (R17);
   * frame period per RSPI (R4) — the probe cycles RSPI 0..3 with `--all-rspi`;
   * GNFD→PDAT-header delay min/mean → δ_sensor (R5); fill `radar.yaml: sensor_delay_s` and set `sensor_delay_source: measured`;
   * header-timestamp jitter p99 (R18) → `usb_latency_s`;
   * early-poll experiment result (R15);
   * per-frame target count, cap-hit rate on a static bench and on a short ride (R7, R10).
3. Kill the probe without GBYE (`kill -9`), rerun: it must recover the baud (R14).
4. Moving reflector (a person walking is fine at RSPI 0/1; set `params.RSPI` per run):
   * **Azimuth sign**: reflector on the rider's LEFT of the rear-facing sensor ⇒ `angle_raw > 0` ⇒ decoded `y < 0` (R1). If the sign is opposite: **stop and ask**.
   * **Doppler sign**: approaching ⇒ `speed_raw < 0` (R2).
   * **Range scale**: tape measure vs `distance_cm` at 3 / 10 / 20 m (R3).
   * **Blind band**: slowest speed that still produces a target (R11) → `clutter.doppler_blind_band_mps`.
   * **Magnitude vs range** with the same reflector at 2 / 5 / 10 / 20 m → recalibrate `clutter.alias_check.c_alias_db` (probe logs `magnitude_raw` per target).
5. Cover the sensor with the vest fabric: does the frame go empty or does flapping create targets (R16)?

## B. IMU (BNO085 over SPI)
1. `sudo raspi-config` → Interface → SPI enable (or `dtparam=spi=on` in `/boot/firmware/config.txt`), reboot; check `/dev/spidev0.0`.
2. Wire CS, INT, RESET to the GPIOs given in `config/pipeline.yaml: imu.pins`; verify with `gpioinfo`.
3. `.venv/bin/python tools/bno085_probe.py --seconds 30`: achieved gyro/accel rates and timestamp jitter (I1). **If gyro < 100 Hz or jitter p99 > 5 ms: stop and report.**
4. Static: `+9.81 m/s²` on the radar +Z axis after the extrinsic (I2); note bias/noise (I4, I5) → `imu.eskf` noise densities.
5. Six-orientation static test (radar +X up/down, +Y up/down, +Z up/down): fill `frames.yaml: imu.q_wxyz`, set `source: measured` (I3).

## C. Haptics (PWM vibration motors on the Pi's GPIOs, `docs/haptics.md`)
1. Each motor through a driver that takes its current from 5 V, not from the pin, with a pull-down on its input (H3). Set the pins and sides in `pipeline.yaml: haptics.motors`.
2. `sudo systemctl stop goldenfleece`, then `.venv/bin/python tools/haptics_test.py`: every motor claimed, its pin high in 10/10 samples while on (100 % duty, H2) and low once off, and felt on the side the config gives it (H1). Then `--patterns` on the body (H4).
3. Run the pipeline with `--radar sim:overtake_left_10mps --imu sim`: "warnings offline" ticks at start, then the warning on the LEFT motor (`HAPTICS` log lines).
4. `kill -STOP` the running pipeline: note what the motors do until systemd's watchdog restarts it and how long until they are off (H5); `kill -9` with a motor on: off after `ExecStopPost` (H7).

## D. Power
1. Load the 5 V rail until `vcgencmd get_throttled` shows 0x1: UNDERVOLTAGE reaches the haptics (degraded marker) within ~1 s (P1).
2. The motors run from the Pi's 5 V rail: a brownout silences them (documented limitation, H6).

## E. First ride
1. `allow_unmeasured: false`; recording on.
2. Afterwards: `tools/outage_report.py`, `tools/replay.py --twice`, and the probe's cap-hit rate → decide the §15.3 question (`DEDI`/`THOF`).

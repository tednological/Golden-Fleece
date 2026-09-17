# Field testing

What runs on the Pi, how to reach it from a phone, what to check before a ride, and what to do after one.
This is a test rig, not a safety device yet: see "Known limits" at the end.

## What runs

| Service | What it does | Unit |
|---|---|---|
| `goldenfleece` | The pipeline: K-LD7 + BNO085 → l03..l09 → MCU link. Records every session to `~/golden_fleece_recordings`. | `deploy/goldenfleece.service` |
| `goldenfleece-web` | The field web app on port 8080. Follows the newest recording, replays l03..l09 for display, and shows the pipeline's own decisions. | `deploy/goldenfleece-web.service` |

Both start at boot. They are separate processes, so the web app cannot slow down or stop the pipeline.
No MCU board is attached yet, so nothing vibrates or lights up: the phone page is the only display.
`MCU_LINK_DOWN` is informational and does not change health or warnings.

## Reaching the page

1. Turn on the phone's hotspot. The Pi joins a saved profile on its own (autoconnect on).
2. Open **http://raspberrypi.local:8080/** in Safari on the phone. If `.local` does not resolve, use the Pi's
   address on the hotspot (`hostname -I` on the Pi; it changes between sessions) or its VPN address if you use one.

The header shows: **connected** (the page is receiving updates), **health** (OK / DEGRADED / OFFLINE, from the
pipeline's decisions), **age** ("live · N ms behind"; "stale" means the pipeline is not recording) and the
session file. The banner is the pipeline's current warning with its side and arrival bucket.

## Service control

```bash
sudo systemctl status goldenfleece goldenfleece-web      # both should be "active (running)"
journalctl -u goldenfleece -f                            # pipeline log: health transitions, link, latency
sudo systemctl stop goldenfleece                         # before bench probes: it owns the radar port and the IMU
sudo systemctl start goldenfleece
sudo systemctl disable goldenfleece goldenfleece-web     # stop starting at boot
```

Running `tools/run_pipeline.py`, `tools/kld7_probe.py` or `tools/bno085_probe.py` by hand while the service is
up makes two processes fight over the radar and the IMU. Stop the service first.

## Before the first ride: bench accuracy checks

About 15 minutes with the page open. Each item is in `docs/UNVERIFIED.md`; record results there with a date.
Stop-and-ask rules apply: if a sign is wrong, report it. Never flip a sign in code.

1. **Azimuth sign (R1).** Face the sensor and walk toward it on *your* left, which is the rider's left. Targets
   must appear on the left of the scope, side LEFT, raw angle > 0.
2. **Doppler sign (R2).** Walking toward the sensor shows v < 0 (orange, approaching); walking away, v > 0.
3. **Range (R3).** Stand at taped 3, 10 and 20 m and step in place: r within about 0.3 m.
4. **Blind band (R11).** The slowest walking pace that still shows a target sets
   `clutter.doppler_blind_band_mps` (placeholder 0.5 m/s).
5. **Alias check (R21, R22).** Currently **off**: its constant was fitted to the simulator and rejected most real
   targets within 5 m. With a reflector moving at 2, 5, 10 and 20 m, read the dB column, set
   `clutter.alias_check.c_alias_db` from it, then re-enable. Until then, a strong car beyond 30 m can appear as a
   ghost at short range.
6. **IMU mount (I2, I3).** Mount the breakout square to the radar. With the vest still, the page's "up axis"
   names the IMU axis pointing up and |a| ≈ 9.8 m/s². `config/frames.yaml` still has the identity rotation
   (nominal), which is only right if the board's X/Y/Z match the radar's (+X rearward, +Y rider's right,
   +Z up). The six-orientation test fills it properly.
7. **Vest fabric (R16).** Cover the sensor with the fabric: flapping must not create targets.

## On the ride

- The pipeline records whether or not the page is open. The phone only watches.
- Watch the header: "stale" means no new data (pipeline stopped or the radar went quiet); DEGRADED lists its
  cause in the banner; the power line in "Health & pipeline" flags under-voltage.
- The IMU card should show the gyro at about 200 Hz. A falling rate or IMU_FAULT means the IMU needs a look.

## After the ride

```bash
.venv/bin/python tools/outage_report.py ~/golden_fleece_recordings/session_*.jsonl
.venv/bin/python tools/replay.py ~/golden_fleece_recordings/<session>.jsonl --twice
.venv/bin/python tools/web_app.py --recording ~/golden_fleece_recordings/<session>.jsonl --port 8081
```

The last one replays a session on the phone at real-time pace (`--speed 4` for faster), on port 8081 so the
live page keeps running.

## Known limits for these tests

- Every threshold is still tagged `unvalidated` in the register (`config/pipeline.yaml`).
- Azimuth sign, Doppler sign and range scale are unconfirmed until the bench checks above pass.
- The alias check is off (above).
- No MCU: no haptics or lights, and a Pi brownout silences everything (documented limitation M5).
- The web view lags the pipeline by the recording flush period (0.2 s) plus up to one page refresh (0.125 s).
- IMU timing under the running service (2026-09-15): gyro 199 Hz, jitter p99 3.2 ms against a 5 ms limit, with
  occasional single gaps up to about 30 ms. The pipeline uses about 21 % of one core (mostly the IMU poll),
  the web app about 9 %.
- Anything moving toward the sensor within 4 m raises at least a WARNING (the proximity rule), and at under 1 m
  an ALERT, including a person at the bench: 6 episodes in 5 minutes with someone sitting next to it
  (2026-09-15). Keep people more than 2 m away during bench checks, except for deliberate walk tests. Whether
  the rider's own legs or the bike do the same on a ride is unknown until the first recording.
- A slow object that stops being detected is kept for up to about 12 s (dashed rings), so a car pacing the
  rider is not forgotten. Such tracks are held at the rider once their predicted range reaches zero and never
  raise a warning.

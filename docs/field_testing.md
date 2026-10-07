# Field testing

What runs on the Pi, how to reach it from a phone, what to check before a ride, and what to do after one.
This is a test rig, not a safety device yet: see "Known limits" at the end.

## What runs

| Service | What it does | Unit |
|---|---|---|
| `goldenfleece` | The pipeline: K-LD7 + BNO085 → l03..l09 → haptics (the vibration motors on the Pi's GPIOs). Records every session to `~/golden_fleece_recordings`. | `deploy/goldenfleece.service` |
| `goldenfleece-web` | The field web app on port 8080. Follows the newest recording, replays l03..l09 for display, and shows the pipeline's own decisions. | `deploy/goldenfleece-web.service` |
| `goldenfleece-camera` | Records the USB camera into `~/golden_fleece/Camera Footage` unless stopped from the web app (see "Camera" below). | `deploy/goldenfleece-camera.service` |

All three start at boot. They are separate processes, so neither the web app nor the camera can slow down or stop
the pipeline.
There is no MCU: the pipeline drives the vibration motors itself, at 100 % duty (`docs/haptics.md`). If their pins
cannot be driven, `HAPTICS_FAULT` makes health OFFLINE and the page says "WARNINGS OFFLINE": the rider would feel
nothing.

## Reaching the page

1. Turn on the phone's hotspot. The Pi joins a saved profile on its own (autoconnect on).
2. Open **http://raspberrypi.local:8080/** in Safari on the phone. If `.local` does not resolve, use the Pi's
   address on the hotspot (`hostname -I` on the Pi; it changes between sessions) or its VPN address if you use one.

The header shows: **connected** (the page is receiving updates), **health** (OK / DEGRADED / OFFLINE, from the
pipeline's decisions), **age** ("live · N ms behind"; "stale" means the pipeline is not recording) and the
session file. The banner is the pipeline's current warning with its side and arrival bucket.

## Service control

```bash
sudo systemctl status goldenfleece goldenfleece-web goldenfleece-camera   # all should be "active (running)"
journalctl -u goldenfleece -f                            # pipeline log: health transitions, link, latency
journalctl -u goldenfleece-camera -f                     # camera log: files started, stops, camera faults
sudo systemctl stop goldenfleece                         # before bench probes: it owns the radar port and the IMU
sudo systemctl start goldenfleece
sudo systemctl disable goldenfleece goldenfleece-web goldenfleece-camera   # stop starting at boot
```

Running `tools/run_pipeline.py`, `tools/kld7_probe.py`, `tools/bno085_probe.py` or `tools/haptics_test.py` by hand
while the service is up makes two processes fight over the radar, the IMU and the motor pins. Stop the service first.
If a pipeline run by hand was killed with a motor buzzing, `.venv/bin/python tools/haptics_test.py --off` stops it.

## Camera

The USB camera ("HD USB Camera", `32e4:9230`, no microphone) records all the time: the recorder starts at boot and
keeps going unless someone presses **Stop recording** on the page's Camera card. A stop lasts until someone presses
**Start recording**, or until the Pi restarts (every boot records). The header chip shows **● REC** and the running
time while it records; tap it to jump to the card. From a shell:
`.venv/bin/python tools/camera_recorder.py --status` (or `--off` / `--on`).

- Files: `~/golden_fleece/Camera Footage/camera_<start time>_<run id>.mkv`, one per 5 minutes, cut on the clock.
  The run id keeps two runs apart even when the Pi boots with a stale clock, so no file is ever overwritten. The
  first file after a boot may carry the clock's pre-sync time, as the pipeline's session files do.
- Format: the camera's own 1920x1080 MJPEG at 30 fps, copied without re-encoding (0.02 cores; measured next to the
  live pipeline with no change in its latency, where software H.264 took 1.2 cores and raised the pipeline's
  header-to-processing p99 from 1.0 to 2.5 ms). Play the files in VLC or mpv; QuickTime and phone browsers
  cannot play MJPEG in MKV.
- Space: about 22-25 GB an hour, so the card holds roughly 8 hours of footage. Recording **pauses by itself** when
  less than 20 GB is free (the card the pipeline records to must never fill) and resumes once space is freed. The
  page's Camera card shows the free space and hours left. Nothing is ever deleted automatically: copy footage off
  and delete it.
- Pulling the battery loses about the last 2 s of footage: the recorder syncs the open file every 2 s, and an MKV
  cut short still plays.
- Settings (size, frame rate, file length, free-space floor) are in `config/camera.yaml`; restart the
  `goldenfleece-camera` service after changing them.

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
8. **Haptics (H1, H2, H4).** With the service stopped, `.venv/bin/python tools/haptics_test.py --patterns` on the
   vest: every motor passes claim, level and felt, on the right side, and the five patterns are told apart. Start
   the service again: the first thing felt is "warnings offline" (three short ticks every 5 s) until the radar runs.

## On the ride

- The pipeline records whether or not the page is open. The phone only watches.
- Watch the header: "stale" means no new data (pipeline stopped or the radar went quiet); DEGRADED lists its
  cause in the banner; the power line in "Health & pipeline" flags under-voltage.
- The IMU card should show the gyro at about 200 Hz. A falling rate or IMU_FAULT means the IMU needs a look.
- The haptics card shows what the motors render (e.g. "warning left", "warnings offline"), the render thread's
  watchdog (NORMAL, or FALLBACK when the pipeline loop stalled), the decision → motor latency and the recent
  changes. The header chip reads "haptics ok"; "haptics: warnings offline" while health is OFFLINE, "haptics: loop
  stalled" in FALLBACK, and "haptics fault" when the pins cannot be driven. Every change is in the session file as
  a `hap` record: `grep '"k":"hap"' ~/golden_fleece_recordings/<session>.jsonl`.

## After the ride

```bash
.venv/bin/python tools/outage_report.py ~/golden_fleece_recordings/session_*.jsonl
.venv/bin/python tools/replay.py ~/golden_fleece_recordings/<session>.jsonl --twice
.venv/bin/python tools/web_app.py --recording ~/golden_fleece_recordings/<session>.jsonl --port 8081
```

The last one replays a session on the phone at real-time pace (`--speed 4` for faster), on port 8081 so the
live page keeps running.

Camera footage from the ride is in `~/golden_fleece/Camera Footage`; match it to the session by the start time
in the file names. Copy it off (for example `scp -r "raspberrypi.local:golden_fleece/Camera Footage" .` on a
laptop; the quotes matter, the folder name has a space) and delete what you do not need: at 22-25 GB an hour the
card fills after about 8 hours of footage, and recording then pauses.

## Known limits for these tests

- Every threshold is still tagged `unvalidated` in the register (`config/pipeline.yaml`).
- Azimuth sign, Doppler sign and range scale are unconfirmed until the bench checks above pass.
- The alias check is off (above).
- No device independent of the Pi: if the whole pipeline process hangs, the motors keep their state (silent, or
  one left on) until systemd restarts it, about 2 s (H5); a Pi brownout silences everything (H6).
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

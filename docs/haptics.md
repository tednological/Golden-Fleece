# Golden Fleece — haptics

Team decision 2026-10-07: **there is no MCU.** The vibration motors are mounted on the Pi's GPIOs, each through a
driver with a PWM input, and are driven at **100 % duty**. The Pi now does what the ICD (`docs/mcu_icd.md`, in git
history) asked of the firmware: rendering priority, patterns, and falling back to "warnings offline" when the
pipeline stops.

Implemented by `goldenfleece/l10_haptics/`: `patterns.py` (pure: what is rendered and when each motor is on),
`motors.py` (hardware: one PWM output per motor, on or off) and `output.py` (the render thread and its watchdog).
Configured in `config/pipeline.yaml: haptics`. Checked on the bench with `tools/haptics_test.py`.

## 1. Hardware

| Item | Value |
|---|---|
| Motors | one entry per motor in `haptics.motors`: `name`, `pin` (Blinka name, `D12` = BCM GPIO12) and `side` (`LEFT` / `RIGHT`) |
| Default pins | left `D12` (GPIO12, header pin 32), right `D13` (GPIO13, header pin 33). **Nominal: confirm against the wiring** (`haptic_pins` in the register). Both are free on this Pi (the IMU uses GPIO5/24/25 and SPI0) and are the Pi 5's hardware PWM0 pins, should hardware PWM be wanted later. |
| Drive | Blinka `pwmio.PWMOut` (lgpio software PWM on the Pi 5). On = `pwm_duty_cycle` (1.0, i.e. 100 %), off = 0. At 100 % lgpio holds the line high, so `pwm_frequency_hz` does not matter. |
| Current | a GPIO gives a few mA at 3.3 V. Each motor must take its current from its driver (transistor, MOSFET or driver IC), never from the pin. |
| Idle level | the driver's input needs a pull-down, so a pin nobody drives (boot, before the pipeline starts, after a crash) means off |
| Power | the Pi's 5 V rail. A brownout silences the motors along with the Pi (as it did with the MCU: limitation H6) |

## 2. Bench test

```bash
sudo systemctl stop goldenfleece                       # the pipeline owns the motor pins while it runs
.venv/bin/python tools/haptics_test.py                 # each motor alone, then all together; asks what you felt
.venv/bin/python tools/haptics_test.py --patterns      # then plays every pattern below, 6 s each
.venv/bin/python tools/haptics_test.py --pins D20,D21  # other pins, in the config's motor order (bring-up)
.venv/bin/python tools/haptics_test.py --off           # every motor off
sudo systemctl start goldenfleece
```

It drives the motors through the pipeline's own code and checks, per motor:

| Check | Pass | A failure means |
|---|---|---|
| claim | the pin is claimed as a PWM output | wrong pin name, the pipeline (or another tool) still holds it ("GPIO busy"), or no `gpio` group |
| level | `pinctrl get` reads the pad **high in 10 of 10 samples while on** and low in 10 of 10 once off | not high: duty below 100 %, a short, or a driver drawing its current from the pin. Not low: something else drives the pin |
| felt | you felt it, on the side the config gives it | loose motor or driver, no driver power, or the two motors' pins swapped (fix `haptics.motors`, never the code) |

Exit status 0 only if every check that ran passed. `--yes` asks nothing (claim and level only). Ctrl-C switches
everything off.

## 3. What the rider feels

### Priority (first match wins)

| Render | When | Motors |
|---|---|---|
| `FALLBACK_OFFLINE` | the pipeline loop has not advanced for `stall_timeout_s` (§4) | "warnings offline" pattern on every motor; threat and alert dropped |
| `OFFLINE` | the pipeline reports health `OFFLINE` | "warnings offline" pattern on every motor; no threat |
| `DEGRADED` | health `DEGRADED` | the threat pattern, if any, on its side **plus** the degraded marker on every motor |
| `THREAT` | level > 0 | the level's pattern on the threat side's motors |
| `NONE` | nothing | every motor off |

Side → motors: `LEFT` and `RIGHT` drive the motors on that side; `CENTER` and `BOTH` drive every motor (and so does
a side with no motor, so a one-motor vest still warns). **Never a lane.**

### Patterns (`haptics.patterns`; tag `unvalidated` until tried on the body while riding)

A pattern is `count` pulses of `pulse_s`, `gap_s` apart, repeated every `period_s`, always at the full duty cycle.

| Pattern | Used for | Default | On-time |
|---|---|---|---|
| `advisory` | level 1 (ADVISORY) | 1 × 150 ms every 2 s | 7.5 % |
| `warning` | level 2 (WARNING) | 2 × 150 ms, 100 ms apart, every 1 s | 30 % |
| `alert` | level 3, only while l09 asserts the alert channel | 250 ms on, 50 ms off (near continuous) | 83 % |
| `degraded` | marker while health is DEGRADED, every motor | 1 × 50 ms every 10 s | 0.5 % |
| `offline` | "warnings offline", every motor | 3 × 50 ms, 150 ms apart, every 5 s | 3 % |

Requirements, enforced when the config is loaded (the pipeline refuses to start otherwise):

* **A fault never feels like a threat:** every fault pulse (degraded, offline) is shorter than every threat pulse.
  Faults are ticks on every motor; threats are pulses on the threat's side.
* **Levels are distinct:** on-time rises advisory < warning < alert.
* **Alert is the only maximum-intensity pattern:** nothing else has as much on-time. The alert pattern follows the
  alert channel (`WarningCommand.assert_alert`), which l09 asserts for level 3 only and never for a fault.

Phases: a threat pattern starts, with a pulse, when its level or side changes; the offline pattern and the degraded
marker start, with a tick, when they are entered (OFFLINE ↔ FALLBACK keep the same phase). So a new warning is felt
at once rather than at the next period.

## 4. Heartbeat honesty and fallback

The pipeline loop (`orchestrator/runner.py`) hands its progress counter to the haptics on **every** iteration (every
radar frame, every 50 ms health tick). The haptics have no timer of their own that could make a stopped pipeline
look alive (`tests/test_invariants.py`). The render thread, which redraws the patterns every `render_period_s`
(5 ms) and on every hand-over, applies the rule the MCU used to:

* the counter has not advanced for `stall_timeout_s` (**250 ms**), or no heartbeat arrived at all → `FALLBACK`:
  alert dropped, "warnings offline" on every motor, recorded with cause `HB_STALL` or `HB_ABSENT`;
* it returns to `NORMAL` as soon as the counter advances ("progress resumed").

250 ms is five missed frames: long enough to ride through a garbage-collector pause without a false "offline",
short enough that a car closing at 20 m/s covers 5 m before the rider is told.

What this does **not** cover, because the render thread lives in the same process:

| Failure | What the rider gets |
|---|---|
| pipeline loop stalled, process alive | "warnings offline" within 0.25 s; systemd restarts the pipeline 2 s after its last `WATCHDOG=1` |
| whole process hung (stuck holding the GIL, `kill -STOP`) | the motors keep whatever state they had: silence, or a motor left on (during an alert, a near-continuous buzz). systemd's watchdog stops the process after 2 s, `SIGKILL`s it if it does not exit, then `ExecStopPost` switches every motor off. **Limitation H5.** |
| process killed or crashed | `ExecStopPost=tools/haptics_test.py --off` switches every motor off; the restarted pipeline renders "warnings offline" until its first radar frame |
| kernel hang | the BCM hardware watchdog (`RuntimeWatchdogSec`, `docs/deployment.md`) reboots the Pi; the pins come up undriven and the drivers' pull-downs keep the motors off |
| brownout | silence (limitation H6) |

A hardware fix for H5 would be a driver that needs a toggling input (AC coupled), so a stuck-high pin cannot hold a
motor on; that is incompatible with a plain 100 % duty level and is a team decision.

## 5. Health, recording, web app

* `HAPTICS_FAULT` (bit 0x200, formerly `MCU_LINK_DOWN`): a motor pin cannot be claimed or written. It is an
  **OFFLINE** reason: with no motors the rider gets no warnings, so the pipeline says so (the phone page shows
  "WARNINGS OFFLINE"). The render thread releases every motor and reclaims them with backoff
  (`reopen_backoff_s`); the bit clears when that succeeds. Old recordings' `MCU_LINK_DOWN` events were
  informational and are ignored by `tools/outage_report.py`.
* Restart: the pipeline hands `PIPELINE_RESTARTING` (OFFLINE) and a heartbeat to the haptics before any decision,
  so the first thing felt after a (re)start is "warnings offline", until the first radar frame is processed.
* Every change in what is rendered is a `hap` record in the session file (render, level, side, alert, health, mode,
  the cause of a FALLBACK change, and `lat`, the decision → motor-write latency, when a new decision caused it), and
  the current state is recorded again every `recording.haptics_refresh_s` (5 s) without a change (`rf: 1`). The
  pipeline log has one `HAPTICS …` line per change. The field web app's haptics card shows them.
* `tools/run_pipeline.py --no-haptics` drives no GPIO; the log and the recording still show what would be felt.

## 6. Timing

| Path | Value | Tag |
|---|---|---|
| decision handed over → motor written | p50 0.04 ms, p99 0.5 ms, max 4.7 ms (render thread, no GPIO cost; this Pi 5, 2026-10-07) | measured (bench, no load) |
| pattern resolution | `render_period_s` = 5 ms | definition |
| loop stall → "warnings offline" | ≤ 250 ms + 5 ms | definition; `tests/test_runner_integration.py` |

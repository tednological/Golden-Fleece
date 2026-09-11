# Golden Fleece — MCU Interface Control Document (ICD)

Version 1 of the `GF1` protocol. Implemented on the Pi side by `goldenfleece/l10_mcu_link/protocol.py` (framing) and `link.py` (transport), and by the reference emulator `goldenfleece/l10_mcu_link/emulator.py`. This document is sufficient to write the MCU firmware without reading the Python.

Team decisions this ICD reflects (Stage 0 answers): the MCU is an **ATmega328P** (Xplained Mini, USB CDC ACM through its mEDBG bridge); the **serial link is the alert channel** and the firmware matches **keywords**; the MCU is **powered through the Pi**.

---

## 1. Physical and transport layer

| Item | Value |
|---|---|
| Pi-side device | `/dev/serial/by-id/usb-ATMEL_mEDBG_CMSIS-DAP_…-if01` (USB CDC ACM), from `config/pipeline.yaml: link.port` |
| MCU-side | ATmega328P UART0 via the mEDBG virtual COM port |
| Baud | **57600 8N1** (config `link.baudrate`). At 16 MHz with U2X the 328P's 57600 error is 0.8 %; 115200 would be 2.1 %, marginal for 8N1. |
| Direction | Full duplex. Pi → MCU: `ALERT WARN HB RCFG HLTH`. MCU → Pi: `STAT OUTG`. |
| Line length | ≤ **96 bytes** including the terminating `\n` (MCU buffer 96 B). |
| Character set | 7-bit ASCII. No binary. |

The link is transport-agnostic: the same lines work over a hardware UART.

## 2. Line format

```
$GF1,<TYPE>,<seq>,<field1>,<field2>,...*<CRC>\n
```

| Element | Rule |
|---|---|
| `$` | start of line. A receiver resynchronises by discarding bytes until `$`. |
| `GF1` | protocol id and version. Reject other ids. |
| `TYPE` | message keyword (§4). |
| `seq` | 0–65535, increments per sender per line, wraps. Informational (duplicate detection, logs). |
| fields | comma-separated, no spaces, no `$ * ,` inside a field. |
| `*CRC` | `*` followed by 4 uppercase hex digits: **CRC-16/CCITT-FALSE** (poly 0x1021, init 0xFFFF, no reflection, no final XOR) over every byte **between** `$` and `*`, exclusive. Check value: `"123456789"` → `29B1`. |
| `\n` | terminator. A `\r` before it is tolerated. |

A line whose CRC does not match, that exceeds 96 bytes, or whose id/type is unknown is **discarded and counted**; it never changes state.

Reference CRC (C):
```c
uint16_t crc16_ccitt_false(const uint8_t *d, size_t n) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < n; i++) {
        crc ^= (uint16_t)d[i] << 8;
        for (uint8_t b = 0; b < 8; b++) crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}
```

## 3. Keywords the firmware may match without full parsing

The firmware is free to react to keywords before parsing the rest of the line, provided it **still verifies the CRC before acting**:

| Keyword | Where | Meaning |
|---|---|---|
| `ALERT` | TYPE | alert channel (§5). `ALERT,<seq>,1` assert, `ALERT,<seq>,0` release |
| `WARN` | TYPE | threat level + side |
| `HB` | TYPE | heartbeat with progress counter (§6) |
| `OFFLINE` / `DEGRADED` / `OK` | health field of WARN, HB, HLTH | health state words (§7) |

Levels are numbers (0–3), never words, so the only occurrence of `ALERT` on a line is the alert-channel message.

## 4. Message catalogue

### 4.1 Pi → MCU

**`ALERT`** — `$GF1,ALERT,<seq>,<state>*CRC`
| field | type | meaning |
|---|---|---|
| state | 0/1 | 1 = assert the alert channel, 0 = release |

**`WARN`** — `$GF1,WARN,<seq>,<level>,<side>,<bucket>,<health>,<bits>,<pseq>,<tms>*CRC`
| field | type | meaning |
|---|---|---|
| level | 0..3 | 0 NONE, 1 ADVISORY, 2 WARNING, 3 ALERT (the threat level; `ALERT` keyword is not used here) |
| side | L/C/R/B/N | coarse side: LEFT, CENTER, RIGHT, BOTH, NONE. **Never a lane.** |
| bucket | 0..4 | time-to-arrival bucket: 0 none, 1 > 6 s, 2 3–6 s, 3 1.5–3 s, 4 < 1.5 s. Arrival at the rider's longitudinal position, not a collision time. |
| health | word | `OK` / `DEGRADED` / `OFFLINE` |
| bits | hex | health reason bits (§7) |
| pseq | u32 | pipeline decision sequence number; echo it in `STAT` |
| tms | u32 | Pi monotonic milliseconds of the decision (latency measurement only) |

**`HB`** — `$GF1,HB,<seq>,<loop>,<frames>,<lastfn>,<health>,<bits>*CRC`
| field | type | meaning |
|---|---|---|
| loop | u32 | **pipeline loop progress counter** (advances on every processed radar frame *and* on every health tick) |
| frames | u32 | radar frames fully processed since start |
| lastfn | u32 | last radar frame number (from the sensor's DONE) |
| health, bits | | as in WARN |

**`RCFG`** — `$GF1,RCFG,<seq>,<rrai>,<rspi>,<baud>,<blind_cms>,<t_alert_ms>,<t_warn_ms>*CRC`
Radar configuration in effect (range index, speed index, sensor baud), the Doppler blind band in cm/s, and the threat time thresholds. Sent on connect, on every change, and every 5 s. Informational for the MCU under `PI_POLLS` (§9).

**`HLTH`** — `$GF1,HLTH,<seq>,<health>,<bits>,<onset_ms>*CRC`
Sent on every health transition and every 1 s. `onset_ms` = age of the oldest active reason.

### 4.2 MCU → Pi

**`STAT`** — `$GF1,STAT,<seq>,<mode>,<mcu_ms>,<last_pseq>,<hb_age_ms>,<outages>*CRC`
| field | meaning |
|---|---|
| mode | `NORMAL` or `FALLBACK` |
| mcu_ms | MCU milliseconds since boot |
| last_pseq | pseq of the last WARN applied (the Pi measures WARN-write → STAT-receive as a latency upper bound) |
| hb_age_ms | milliseconds since the last valid HB |
| outages | count of outage-log entries since boot |

Send `STAT` at ≥ 2 Hz and immediately after applying a WARN.

**`OUTG`** — `$GF1,OUTG,<seq>,<start_ms>,<dur_ms>,<cause>*CRC`
One line per closed outage-log entry (§8). Cause words: `HB_ABSENT HB_STALL ALERT_STALE PI_OFFLINE PI_DEGRADED`.

### 4.3 Sequencing from the Pi

Per pipeline loop iteration the Pi sends, in this order and only what changed or is due: `ALERT` (if the alert state changed, or as a refresh while asserted), `WARN` (on change or every 200 ms), `HB` (every iteration, ≥ 50 ms apart), `HLTH` (on change or every 1 s), `RCFG` (on change or every 5 s). Newer messages of a kind supersede queued older ones on the Pi side; the MCU always sees the latest state.

## 5. Alert channel (keyword `ALERT`)

The alert channel is the UART line `ALERT,<seq>,1`. Semantics identical to a dedicated GPIO line, with one consequence stated in §9.

| Rule | Value |
|---|---|
| Asserted for | threat level 3 only. **Never for a fault.** (l09 guarantees this; the emulator asserts it.) |
| Ordering | the Pi sends `ALERT,1` **before** the `WARN` of the same decision |
| Refresh | while asserted, the Pi repeats `ALERT,1` with every heartbeat (≤ 50 ms) |
| Release | `ALERT,0` once, when the level drops below 3 |
| **Stale (stuck) alert**, `T_line_max` | if no `ALERT,1` refresh arrives for **300 ms**, or the heartbeat is absent/stalled, the MCU **drops the alert and logs `ALERT_STALE`** — a fault, not a threat |

On receipt of `ALERT,1` the MCU may start the maximum-threat pattern immediately, before the WARN arrives.

## 6. Heartbeat honesty and MCU fallback

The heartbeat is generated **by the pipeline loop** (the same code that processes radar frames). There is no independent heartbeat timer on the Pi. Consequently:

* if the loop is running, `loop` advances (every ≤ 50 ms);
* if the process is alive but the pipeline is deadlocked, `HB` lines may still arrive with the **same** `loop` value — the MCU must treat that as a stall;
* if the Pi hangs, crashes, browns out, or the USB link drops, `HB` stops.

**Fallback rule.** The MCU enters `FALLBACK` when either
* no valid `HB` for **250 ms** (`T_absent`), or
* `loop` has not advanced for **250 ms** (`T_stall`).

It returns to `NORMAL` when a valid `HB` with an advanced `loop` arrives.

**Timing justification.** Heartbeats arrive every 29 ms (frame-driven) or 50 ms (health ticks). 250 ms = five missed heartbeats: long enough to ride through USB and garbage-collector hiccups of up to ~200 ms without a false "offline", short enough that a car closing at 20 m/s covers only 5 m before the rider is told. Worst case from Pi hang to the rider hearing "warnings offline": **250 ms + MCU render latency (≤ 20 ms) ≈ 0.3 s**.

## 7. Health and rendering

Health words: `OK`, `DEGRADED`, `OFFLINE`. Reason bits (hex, ORed):

| bit | name | state |
|---|---|---|
| 0x001 | RADAR_SILENT | OFFLINE |
| 0x002 | RADAR_GAPS | DEGRADED |
| 0x004 | RADAR_POSSIBLY_BLOCKED | DEGRADED (advisory) |
| 0x008 | PDAT_SATURATED | DEGRADED |
| 0x010 | IMU_FAULT | DEGRADED (warnings continue) |
| 0x020 | EGO_INVALID | informational |
| 0x040 | UNDERVOLTAGE | DEGRADED |
| 0x080 | THROTTLED | DEGRADED |
| 0x100 | PIPELINE_RESTARTING | OFFLINE |
| 0x200 | MCU_LINK_DOWN | informational (Pi-side only) |
| 0x400 | RADAR_CONFIG_MISMATCH | OFFLINE |

**Rider-perceivable classes** (patterns are the firmware's design; the distinctions are requirements):

| Class | When | Requirement |
|---|---|---|
| THREAT level 1/2/3 with side | `NORMAL` mode, health `OK`, level > 0 | levels perceptually distinct; level 3 is the only maximum-intensity pattern |
| THREAT + DEGRADED marker | `NORMAL`, health `DEGRADED` | the threat pattern is still rendered; a distinct low-salience marker indicates degraded |
| OFFLINE ("warnings offline") | `NORMAL`, health `OFFLINE` | no threat pattern; a distinct fault pattern |
| FALLBACK OFFLINE ("warnings offline") | MCU in `FALLBACK` | same fault pattern as OFFLINE; alert dropped |

**A fault is never rendered as the maximum threat.** Faults must not resemble any threat pattern (a startled swerve is the failure to avoid).

### Truth table: alert × heartbeat

| Alert channel | Heartbeat | MCU behaviour |
|---|---|---|
| released | alive | render per WARN level and health |
| asserted (refreshed < 300 ms) | alive | maximum threat pattern (level 3) |
| asserted, no refresh > 300 ms | alive | **fault**: drop alert, log `ALERT_STALE`, render per last WARN/health |
| asserted | progress-stalled | `FALLBACK`: drop alert, "warnings offline", log `HB_STALL` |
| asserted | absent | `FALLBACK`: drop alert, "warnings offline", log `HB_ABSENT` |
| released | progress-stalled / absent | `FALLBACK`, "warnings offline" |

## 8. Outage logging on the MCU

Keep a log (RAM ring of ≥ 16 entries, or EEPROM) of every departure from "NORMAL and OK":

| field | meaning |
|---|---|
| start_ms | MCU time at onset |
| dur_ms | duration |
| cause | `HB_ABSENT`, `HB_STALL`, `ALERT_STALE`, `PI_OFFLINE` (Pi reported OFFLINE), `PI_DEGRADED` |

Emit closed entries as `OUTG` lines (on request or when closed) and the count in `STAT`. Together with the Pi-side recordings and `tools/outage_report.py` this turns "how often does it fail?" into a measured number.

## 9. Physical constraints and preconditions (documented, not solved)

1. **`PI_POLLS` topology.** The K-LD7 transmits only in response to the Pi's `GNFD` polls. During a Pi hang the sensor is silent, so a passive tap of the radar TX line would carry nothing. MCU fallback under this topology must treat radar data as unavailable and announce "warnings offline". Topology remains a team decision.
2. **MCU powered through the Pi (team answer 6).** A Pi brownout also removes MCU power, so a brownout cannot be announced by the MCU. The precondition of task §10.3 (independent MCU power or hold-up) is **not met**; the rider will experience silence during a brownout. Recorded as a known limitation in `docs/UNVERIFIED.md`.
3. **Alert over the serial stack (team answer 5).** The alert channel shares the failure domain of the serial link. A USB re-enumeration delays or loses an `ALERT`; the WARN carries the same information, and the stale-alert rule bounds a stuck assertion to 300 ms.
4. **Restart.** On process start the Pi sends `HLTH … OFFLINE,100,…` (PIPELINE_RESTARTING) and an `HB` **before** any `WARN`; the MCU stays OFFLINE until an `HB`/`WARN` reports `OK`/`DEGRADED`.

## 10. Worst-case announcement times (Pi side + link)

| Event | Pi detection | Reaches MCU | Notes |
|---|---|---|---|
| Pi hang / crash | — | ≤ 250 ms (MCU rule) | + render ≤ 20 ms |
| RADAR_SILENT | 5 × 29 ms = 145 ms | + ≤ 50 ms tick + ≤ 10 ms link ≈ **205 ms** | measured in simulation: 170 ms |
| RADAR_GAPS | 1 s window, 20 % | ≈ 1.06 s worst; measured 0.25 s onset in simulation | |
| IMU_FAULT | 100 ms no-sample | ≈ 160 ms; measured 100 ms | |
| PDAT_SATURATED | 1 s window, 50 % | ≈ 1.06 s | |
| RADAR_POSSIBLY_BLOCKED | 5 s window | ≈ 5.06 s; measured 5.02 s | advisory |
| UNDERVOLTAGE / THROTTLED | 1 s poll | ≈ 1.06 s | only if the MCU is still powered |
| PIPELINE_RESTARTING | at start | first line after process start | the MCU is already in FALLBACK |

## 11. Examples

```
$GF1,HB,17,120,118,4711,OK,0*3C1A
$GF1,ALERT,18,1*....
$GF1,WARN,19,3,L,4,OK,0,88,123456*....
$GF1,HLTH,20,DEGRADED,10,250*....
$GF1,STAT,5,NORMAL,98765,88,12,0*....
```
(CRCs abbreviated; compute with §2. `tests/test_l10_protocol_emulator.py` contains verified vectors.)

## 12. Firmware implementation notes (ATmega328P)

* 96-byte line buffer; on `$` reset the buffer; on `\n` verify CRC, then dispatch on TYPE.
* Keep two timers: last valid `HB` time and last `loop` change time; evaluate the §6 rule in the main loop at ≥ 100 Hz.
* Keep the last `ALERT,1` time; evaluate the 300 ms staleness rule likewise.
* Output patterns must be non-blocking (no `delay()` in the parser path).
* Send `STAT` every 500 ms and after each applied `WARN`.

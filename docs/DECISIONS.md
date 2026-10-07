# Team decisions recorded (answers to the Stage 0 questions, 2026-09-11)

| # | Question | Decision | Where it landed |
|---|---|---|---|
| 1 | Which USB adapter carries the K-LD7 | Prolific PL2303 (ATEN Serial Bridge, `067b:23a3`, `/dev/ttyUSB0`) | `config/radar.yaml: port` (by-id path) |
| 2 | MCU | Swapped to **ATmega328P-XMINI** (Atmel, CDC ACM via mEDBG, `03eb:2145`, `/dev/ttyACM1`). **Superseded by 12: no MCU.** | (removed; `docs/mcu_icd.md` in git history) |
| 3 | §15.3 cap-contention mitigation | Leave configurable for testing | `radar.yaml: params.THOF / DEDI / MISP`; probe measures the cap-hit rate |
| 4 | Sensor truncation order above 12 targets | Leave configurable for testing | simulator `RadarModelParams.cap_policy` (`weakest_first`, `fft_order`, `random`); tagged unverified |
| 5 | Alert line | **The UART is the alert line**; the Arduino matches keywords. **Superseded by 12:** the alert channel is `WarningCommand.assert_alert`, rendered as the alert pattern | `docs/haptics.md` §3 |
| 6 | MCU power | Powered through the Pi. **Superseded by 12:** the motors are powered from the Pi's 5 V rail | Brownout silences them: documented limitation (UNVERIFIED H6) |
| 7 | Recording storage | Root card; buffer so recording never delays the loop | `recording.dir = /home/ted/golden_fleece_recordings`; writer thread + bounded queue, drops counted ("use DMA to delay" read as: buffered, non-blocking writes; there is no user-space DMA path for SD writes) |
| 8 | Environment | Create a venv | `.venv` (`--system-site-packages`), scipy / pytest / hypothesis / adafruit packages installed |
| 9 | Receding movers to tracker; UNEXPLAINED coast holds level; heartbeat timing | Leave configurable | `clutter.pass_receding_movers`, `threat.unexplained_coast_holds_level`, `haptics.stall_timeout_s` (was `link.*` before decision 12) |
| 10 | Speed aliasing above 27.8 m/s relative (RSPI 3) | Record as limitation | `docs/UNVERIFIED.md` R19 |
| 11 | δ_sensor method | Use the timing-statistics method | `tools/kld7_probe.py` reports both estimators (free-running min / poll-triggered mean − T) |
| 12 | MCU (2026-10-07) | **Removed.** The haptics are mounted on the Pi: PWM-driven vibration motors on GPIOs, **duty cycle 100 %** | `goldenfleece/l10_haptics/`, `config/pipeline.yaml: haptics`, `docs/haptics.md`, `tools/haptics_test.py`; `HAPTICS_FAULT` replaces `MCU_LINK_DOWN` and is an OFFLINE reason |

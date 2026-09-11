"""Run the reference MCU emulator over a serial port (a pty pair or a second USB adapter) so a Pi build
can be exercised against the ICD behaviour without firmware, or so firmware output can be compared.

    socat -d -d pty,raw,echo=0 pty,raw,echo=0      # gives two ptys; point the pipeline at one, this tool at the other
    .venv/bin/python tools/mcu_emulator_serial.py --port /dev/pts/N
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from goldenfleece.clock import MonotonicClock                 # noqa: E402
from goldenfleece.l10_mcu_link.emulator import McuEmulator    # noqa: E402


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=57600)
    ap.add_argument("--stat-period", type=float, default=0.5)
    a = ap.parse_args(argv)
    import serial
    ser = serial.Serial(a.port, a.baud, timeout=0.02)
    clk = MonotonicClock()
    mcu = McuEmulator(clk)
    last_stat = clk.now()
    n_render = 0
    print("emulator listening on", a.port)
    try:
        while True:
            data = ser.read(256)
            if data:
                mcu.feed(data)
            mcu.poll()
            if len(mcu.render_log) != n_render:
                n_render = len(mcu.render_log)
                t, render, level, side, alert, health = mcu.render_log[-1]
                print(f"{t - mcu.t_start:8.3f}s RENDER {render.value:16s} level={level} side={side} alert={alert} health={health} mode={mcu.mode.value}")
            if clk.now() - last_stat >= a.stat_period:
                ser.write(mcu.status_line())
                for line in mcu.outage_lines():
                    ser.write(line)
                mcu.outages = [o for o in mcu.outages if o.end_s is None]
                last_stat = clk.now()
    except KeyboardInterrupt:
        pass
    print("crc errors:", mcu.n_crc_errors, "messages:", mcu.n_messages, "outages:", len(mcu.outages))
    return 0


if __name__ == "__main__":
    sys.exit(main())

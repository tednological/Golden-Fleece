"""Terminal "vest": runs the reference MCU emulator (goldenfleece/l10_mcu_link/emulator.py) on a pseudo-terminal
and prints every change in what the vest would render.  A stand-in for socat + tools/mcu_emulator_serial.py
when no MCU firmware exists yet.

    .venv/bin/python tools/vest_display.py /tmp/gf_vest           # terminal 1
    .venv/bin/python tools/run_pipeline.py --link-port /tmp/gf_vest # terminal 2

The pty's slave end is exposed as a symlink at the given path; the pipeline opens it like the real board's
serial port, so the real link code, framing and CRC run end to end.
"""
from __future__ import annotations

import os
import select
import signal
import sys
import tty
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from goldenfleece.clock import MonotonicClock                 # noqa: E402
from goldenfleece.l10_mcu_link.emulator import McuEmulator    # noqa: E402


def main(argv=None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) != 1:
        print(__doc__)
        return 2
    link_path = argv[0]
    master, slave = os.openpty()
    tty.setraw(slave)            # keep the slave open too, so the master never sees EIO while the pipeline reconnects
    if os.path.lexists(link_path):
        os.unlink(link_path)
    os.symlink(os.ttyname(slave), link_path)

    stop = {"flag": False}

    def _stop(*_):
        stop["flag"] = True
    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT, _stop)

    clk = MonotonicClock()
    mcu = McuEmulator(clk)
    print(f"vest display listening on {link_path} -> {os.ttyname(slave)}", flush=True)
    n_render, last_stat = 0, clk.now()
    try:
        while not stop["flag"]:
            if select.select([master], [], [], 0.02)[0]:
                mcu.feed(os.read(master, 4096))
            mcu.poll()
            if len(mcu.render_log) != n_render:
                n_render = len(mcu.render_log)
                t, render, level, side, alert, health = mcu.render_log[-1]
                print(f"{t - mcu.t_start:8.3f}s VEST {render.value:16s} level={level} side={side} alert={alert} "
                      f"health={health} mode={mcu.mode.value}", flush=True)
            if mcu.n_messages and clk.now() - last_stat >= 0.5:
                os.write(master, mcu.status_line())
                for line in mcu.outage_lines():
                    os.write(master, line)
                mcu.outages = [o for o in mcu.outages if o.end_s is None]
                last_stat = clk.now()
    finally:
        os.unlink(link_path)
    print("crc errors:", mcu.n_crc_errors, "messages:", mcu.n_messages, "outages:", len(mcu.outages), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())

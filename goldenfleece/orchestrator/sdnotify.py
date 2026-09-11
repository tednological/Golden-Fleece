"""systemd notifications (READY=1, WATCHDOG=1, STATUS=...) over NOTIFY_SOCKET, stdlib only.

WATCHDOG=1 is sent by the ORCHESTRATOR LOOP on progress (same honesty rule as the heartbeat);
nothing here runs a timer.  Absent NOTIFY_SOCKET, every call is a no-op.
"""
from __future__ import annotations

import os
import socket
from typing import Optional


class SdNotifier:
    def __init__(self, addr: Optional[str] = None) -> None:
        self.addr = addr if addr is not None else os.environ.get("NOTIFY_SOCKET")
        self._sock: Optional[socket.socket] = None
        self.sent = 0
        self.errors = 0
        if self.addr:
            try:
                self._sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
            except OSError:
                self._sock = None

    @property
    def enabled(self) -> bool:
        return self._sock is not None

    def _send(self, msg: str) -> None:
        if self._sock is None:
            return
        addr = self.addr
        if addr.startswith("@"):
            addr = "\0" + addr[1:]
        try:
            self._sock.sendto(msg.encode("utf-8"), addr)
            self.sent += 1
        except OSError:
            self.errors += 1

    def ready(self) -> None:
        self._send("READY=1")

    def watchdog(self) -> None:
        self._send("WATCHDOG=1")

    def status(self, text: str) -> None:
        self._send(f"STATUS={text}")

    def stopping(self) -> None:
        self._send("STOPPING=1")

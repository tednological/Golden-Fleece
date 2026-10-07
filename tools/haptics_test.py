"""Haptics bench test: is every vibration motor wired, on the side the config says, and driven at 100 % duty?

    sudo systemctl stop goldenfleece                       # the pipeline owns the motor pins while it runs
    .venv/bin/python tools/haptics_test.py                 # each motor alone, then all together; asks what you felt
    .venv/bin/python tools/haptics_test.py --patterns      # then play every pattern the pipeline renders
    .venv/bin/python tools/haptics_test.py --yes           # no questions: run the checks and print what happens
    .venv/bin/python tools/haptics_test.py --pins D12,D13  # other pins than the config (in the config's motor order)
    .venv/bin/python tools/haptics_test.py --off           # every motor off, then exit (the service's ExecStopPost)

Pins, sides, duty cycle and patterns come from config/pipeline.yaml (``haptics``; only that file is read, so a radar
config problem never stops ``--off``).  The motors are driven through the pipeline's own code
(goldenfleece/l10_haptics), so a pass here means the pipeline can drive them too.

Checks, per motor:
  claim  the pin can be claimed as a PWM output (right name, not in use by another process, permissions);
  level  while the motor is on, the pin reads high in every sample of ``pinctrl get`` (the pad level the SoC sees:
         100 % duty, nothing pulling it down) and low in every sample once it is off.  Skipped without pinctrl;
  felt   you felt it buzz, on the rider's side the config gives it (asked; skipped with --yes).
Exit status 0 only if every check that ran passed.  Every motor is switched off on exit, Ctrl-C included.
"""
from __future__ import annotations

import argparse
import dataclasses
import re
import shutil
import signal
import subprocess
import sys
from pathlib import Path
from typing import Callable, List, Optional

import yaml

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from goldenfleece.clock import Clock, MonotonicClock                                   # noqa: E402
from goldenfleece.config import ConfigError, Section                                   # noqa: E402
from goldenfleece.l10_haptics.motors import BlinkaPwmMotor, MotorPort                  # noqa: E402
from goldenfleece.l10_haptics.patterns import (FAULT_PATTERNS, THREAT_PATTERNS, HapticMode, HapticsConfig,   # noqa: E402
                                               Motor, Renderer, render_state)
from goldenfleece.types import HealthState, ThreatLevel                                # noqa: E402

LEVEL_SAMPLES = 10
SETTLE_S = 0.05                  # after a switch, before the pin level is sampled
BUSY_HINT = ("is the pipeline running? It owns the motor pins: sudo systemctl stop goldenfleece "
             "(and stop any tools/run_pipeline.py or run_preset_*.sh)")
PATTERN_DEMO = (  # (pattern, level, alert, health, what the rider should feel)
    ("advisory", ThreatLevel.ADVISORY, False, HealthState.OK, "level 1: something approaching, far away"),
    ("warning", ThreatLevel.WARNING, False, HealthState.OK, "level 2: approaching"),
    ("alert", ThreatLevel.ALERT, True, HealthState.OK, "level 3: closing fast; the only near-continuous pattern"),
    ("degraded", ThreatLevel.NONE, False, HealthState.DEGRADED, "degraded marker: warnings continue, a sensor is impaired"),
    ("offline", ThreatLevel.NONE, False, HealthState.OFFLINE, "warnings offline: a fault, never a threat"),
)
assert {p for p, *_ in PATTERN_DEMO} == set(THREAT_PATTERNS + FAULT_PATTERNS)


# --- config ----------------------------------------------------------------------------------------------------
def load_haptics(config_dir: Path, pins: Optional[str] = None) -> HapticsConfig:
    data = yaml.safe_load((Path(config_dir) / "pipeline.yaml").read_text(encoding="utf-8"))
    if not isinstance(data, dict) or "haptics" not in data:
        raise ConfigError(f"{config_dir}/pipeline.yaml has no 'haptics' section")
    from goldenfleece.l10_haptics.patterns import haptics_config
    cfg = haptics_config(Section(data["haptics"], "haptics"))
    if pins:
        names = [p.strip() for p in pins.split(",") if p.strip()]
        if len(names) != len(cfg.motors):
            raise ConfigError(f"--pins gives {len(names)} pins for {len(cfg.motors)} motors "
                              f"({', '.join(m.name for m in cfg.motors)}, in that order)")
        cfg = dataclasses.replace(cfg, motors=tuple(dataclasses.replace(m, pin=p) for m, p in zip(cfg.motors, names)))
    return cfg


def bcm_number(pin: str) -> Optional[int]:
    m = re.fullmatch(r"D(\d+)", pin)
    return int(m.group(1)) if m else None


# --- pin level readback ------------------------------------------------------------------------------------------
def pinctrl_level(gpio: int) -> Optional[bool]:
    """The pad level of BCM ``gpio`` as ``pinctrl get`` reports it ("12: op dh pd | hi // GPIO12 = output").
    None if pinctrl is missing or prints no level."""
    try:
        out = subprocess.run(["pinctrl", "get", str(gpio)], capture_output=True, text=True, timeout=2.0).stdout
    except (OSError, subprocess.SubprocessError):
        return None
    m = re.search(r"\|\s*(hi|lo)\b", out)
    return None if m is None else m.group(1) == "hi"


def default_level_reader() -> Optional[Callable[[int], Optional[bool]]]:
    return pinctrl_level if shutil.which("pinctrl") else None


# --- the test ----------------------------------------------------------------------------------------------------
@dataclasses.dataclass
class MotorResult:
    motor: Motor
    claim_error: Optional[str] = None
    level: Optional[str] = None         # "10/10 high, 10/10 low"; None = not checked
    level_ok: Optional[bool] = None
    felt: Optional[bool] = None         # None = not asked

    @property
    def passed(self) -> bool:
        return self.claim_error is None and self.level_ok is not False and self.felt is not False


class Bench:
    def __init__(self, cfg: HapticsConfig, motor_factory: Callable[[Motor], MotorPort], clock: Clock,
                 level_reader: Optional[Callable[[int], Optional[bool]]], ask: Optional[Callable[[str], str]],
                 out: Callable[[str], None] = print) -> None:
        self.cfg = cfg
        self.factory = motor_factory
        self.clock = clock
        self.read_level = level_reader
        self.ask = ask
        self.out = out
        self.ports: List[Optional[MotorPort]] = [None] * len(cfg.motors)
        self.results = [MotorResult(m) for m in cfg.motors]

    # -- motors ---------------------------------------------------------------------------------------------------
    def claim_all(self) -> None:
        for i, m in enumerate(self.cfg.motors):
            try:
                port = self.factory(m)
                port.set(False)
            except Exception as e:  # noqa: BLE001  (MotorError, or whatever a factory raises)
                self.results[i].claim_error = str(e)
                self.out(f"  {m.name:6s} {m.pin:4s} cannot be claimed: {e}")
                if "busy" in str(e).lower():
                    self.out(f"         {BUSY_HINT}")
                continue
            self.ports[i] = port

    def set_all(self, states: List[bool]) -> None:
        for port, on in zip(self.ports, states):
            if port is not None:
                port.set(on)

    def all_off_and_release(self) -> None:
        for i, port in enumerate(self.ports):
            if port is None:
                continue
            try:
                port.set(False)
            except Exception as e:  # noqa: BLE001  (close() below still writes 0 and releases)
                self.out(f"  {self.cfg.motors[i].name}: switching off failed ({e}); releasing the pin")
            finally:
                port.close()
                self.ports[i] = None

    # -- checks ---------------------------------------------------------------------------------------------------
    def _sample(self, gpio: int) -> List[Optional[bool]]:
        return [self.read_level(gpio) for _ in range(LEVEL_SAMPLES)]

    def _question(self, text: str) -> Optional[bool]:
        """y -> True, n -> False, r -> None (the caller repeats the check)."""
        while True:
            a = self.ask(f"  {text} [y/n/r = repeat] ").strip().lower()
            if a in ("y", "yes"):
                return True
            if a in ("n", "no"):
                return False
            if a in ("r", "repeat"):
                return None

    def check_motor(self, i: int, on_s: float) -> None:
        r, m, port = self.results[i], self.cfg.motors[i], self.ports[i]
        self.out(f"\n[{i + 1}/{len(self.cfg.motors)}] {m.name} · {m.pin} · rider's {m.side.name}")
        if port is None:
            self.out(f"  skipped: {r.claim_error}")
            return
        gpio = bcm_number(m.pin)
        while True:
            self.out(f"  ON for {on_s:g} s at {self.cfg.duty_cycle * 100:.0f} % duty …")
            t0 = self.clock.now()
            port.set(True)
            self.clock.sleep(SETTLE_S)
            high = self._sample(gpio) if (self.read_level and gpio is not None) else None
            self.clock.sleep(max(0.0, on_s - (self.clock.now() - t0)))
            port.set(False)
            self.clock.sleep(SETTLE_S)
            low = self._sample(gpio) if high is not None else None
            if high is None or any(v is None for v in high + low):
                why = "pinctrl not found" if self.read_level is None else (
                    f"{m.pin} is not a D<n> pin name" if gpio is None else "pinctrl printed no level")
                self.out(f"  pin level: not checked ({why})")
            else:
                n_hi, n_lo = sum(high), sum(not v for v in low)
                r.level = f"{n_hi}/{LEVEL_SAMPLES} high while on, {n_lo}/{LEVEL_SAMPLES} low while off"
                r.level_ok = n_hi == LEVEL_SAMPLES and n_lo == LEVEL_SAMPLES
                self.out(f"  pin level: {r.level}  {'ok' if r.level_ok else 'FAIL'}")
                if n_hi < LEVEL_SAMPLES:
                    self.out("    not held high: duty below 100 %, a short or an overloaded pin (does the motor's driver "
                             "take its current from the GPIO?)")
                if n_lo < LEVEL_SAMPLES:
                    self.out("    not low once off: something else drives this pin")
            if self.ask is None:
                return
            r.felt = self._question(f"Did you feel the {m.name} motor buzz, on the rider's {m.side.name}?")
            if r.felt is not None:
                return

    def check_together(self, on_s: float) -> Optional[bool]:
        if not any(p is not None for p in self.ports) or len(self.cfg.motors) < 2:
            return None
        while True:
            self.out(f"\n[all] every motor ON together for {on_s:g} s …")
            self.set_all([True] * len(self.ports))
            self.clock.sleep(on_s)
            self.set_all([False] * len(self.ports))
            if self.ask is None:
                return None
            felt = self._question("Did every motor buzz together?")
            if felt is not None:
                return felt

    def play_patterns(self, seconds: float) -> Optional[bool]:
        """Every pattern the pipeline renders, through its own Renderer, on the first motor's side for the threats."""
        if not any(p is not None for p in self.ports):
            return None
        side = self.cfg.motors[0].side
        while True:
            self.out(f"\n[patterns] {seconds:g} s each; threats on the rider's {side.name}, faults on every motor")
            for name, level, alert, health, meaning in PATTERN_DEMO:
                rs = render_state(HapticMode.NORMAL, level, side, alert, health)
                self.out(f"  {name:9s} {self.cfg.patterns[name].text():40s} {meaning}")
                renderer = Renderer(self.cfg.motors, self.cfg.patterns)
                t0 = self.clock.now()
                while (t := self.clock.now() - t0) < seconds:
                    self.set_all(list(renderer.outputs(rs, t)))
                    self.clock.sleep(self.cfg.render_period_s)
                self.set_all([False] * len(self.ports))
                self.clock.sleep(1.0)
            if self.ask is None:
                return None
            distinct = self._question("Could you tell all five apart, and did neither fault pattern feel like a warning?")
            if distinct is not None:
                return distinct


def summary(bench: Bench, together: Optional[bool], patterns: Optional[bool]) -> bool:
    yn = {None: "-", True: "yes", False: "NO"}
    bench.out("\nSummary")
    bench.out(f"  {'motor':8s}{'pin':6s}{'side':7s}{'claim':8s}{'level':8s}felt")
    for r in bench.results:
        level = "-" if r.level_ok is None else ("ok" if r.level_ok else "FAIL")
        bench.out(f"  {r.motor.name:8s}{r.motor.pin:6s}{r.motor.side.name:7s}{'ok' if r.claim_error is None else 'FAIL':8s}"
                  f"{level:8s}{yn[r.felt]}")
    if together is not None:
        bench.out(f"  all together: {yn[together]}")
    if patterns is not None:
        bench.out(f"  patterns distinct: {yn[patterns]}")
    ok = all(r.passed for r in bench.results) and together is not False and patterns is not False
    bench.out("PASS" if ok else "FAIL")
    return ok


def run_off(cfg: HapticsConfig, motor_factory: Callable[[Motor], MotorPort], out: Callable[[str], None] = print) -> bool:
    ok = True
    for m in cfg.motors:
        try:
            port = motor_factory(m)
        except Exception as e:  # noqa: BLE001
            out(f"{m.name} {m.pin}: cannot claim ({e})")
            ok = False
            continue
        try:
            port.set(False)
            out(f"{m.name} {m.pin}: off")
        except Exception as e:  # noqa: BLE001
            out(f"{m.name} {m.pin}: could not switch off ({e})")
            ok = False
        finally:
            port.close()
    return ok


def main(argv=None, motor_factory: Optional[Callable[[Motor], MotorPort]] = None,
         level_reader: Optional[Callable[[int], Optional[bool]]] = None, ask: Optional[Callable[[str], str]] = input,
         clock: Optional[Clock] = None) -> int:
    """``motor_factory``, ``level_reader`` (None: pinctrl, if installed), ``ask`` and ``clock`` are for tests."""
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=str(ROOT / "config"), help="directory holding pipeline.yaml")
    ap.add_argument("--pins", default=None, help="comma-separated Blinka pin names replacing the config's, in its motor order")
    ap.add_argument("--on-s", type=float, default=1.0, help="how long each motor buzzes (default 1 s)")
    ap.add_argument("--patterns", action="store_true", help="also play every pattern the pipeline renders")
    ap.add_argument("--pattern-s", type=float, default=6.0, help="how long each pattern plays (default 6 s)")
    ap.add_argument("--yes", action="store_true", help="ask nothing; only the claim and pin-level checks decide")
    ap.add_argument("--off", action="store_true", help="switch every motor off and exit")
    a = ap.parse_args(argv)
    try:
        cfg = load_haptics(Path(a.config), a.pins)
    except (OSError, ConfigError, yaml.YAMLError) as e:
        print(f"config: {e}", file=sys.stderr)
        return 2
    if motor_factory is None:
        motor_factory = lambda m: BlinkaPwmMotor(m.pin, cfg.frequency_hz, cfg.duty_u16)  # noqa: E731
    if a.off:
        return 0 if run_off(cfg, motor_factory) else 1
    if level_reader is None:
        level_reader = default_level_reader()
    if a.yes:
        ask = None
    elif ask is input and not sys.stdin.isatty():
        print("stdin is not a terminal: not asking what you felt (as with --yes)")
        ask = None

    print(f"Golden Fleece haptics test · {Path(a.config) / 'pipeline.yaml'}")
    print(f"  PWM {cfg.frequency_hz:g} Hz, {cfg.duty_cycle * 100:.0f} % duty while on ({cfg.duty_u16}/65535)")
    for m in cfg.motors:
        print(f"  {m.name:6s} {m.pin:4s} rider's {m.side.name}")
    if level_reader is None:
        print("  pinctrl not found: pin levels will not be checked")

    def _term(signum, frame):
        raise KeyboardInterrupt
    old_term = signal.signal(signal.SIGTERM, _term)
    bench = Bench(cfg, motor_factory, clock or MonotonicClock(), level_reader, ask)
    together = patterns = None
    try:
        bench.claim_all()
        for i in range(len(cfg.motors)):
            bench.check_motor(i, a.on_s)
        together = bench.check_together(a.on_s)
        if a.patterns:
            patterns = bench.play_patterns(a.pattern_s)
    except KeyboardInterrupt:
        print("\ninterrupted: switching every motor off")
        return 130
    finally:
        bench.all_off_and_release()
        signal.signal(signal.SIGTERM, old_term)
    return 0 if summary(bench, together, patterns) else 1


if __name__ == "__main__":
    sys.exit(main())

"""tools/haptics_test.py with motors that need no hardware: it passes when every motor is claimed, held high while on
and felt; it fails on a pin that does not go high, a busy pin or a "no"; --off and Ctrl-C leave every motor off."""
import subprocess

import pytest

from goldenfleece.clock import FakeClock
from goldenfleece.l10_haptics.motors import MotorError, NullMotor
from tools import haptics_test as H

GPIO = {"D12": 12, "D13": 13, "D20": 20, "D21": 21}


class Rig:
    """Motors by pin, and a pin-level reader that sees what they were told (or a stuck level)."""

    def __init__(self, busy=(), stuck=None):
        self.motors, self.busy, self.stuck = {}, set(busy), stuck

    def factory(self, m):
        if m.pin in self.busy:
            raise MotorError(f"{m.pin}: GPIO busy")
        self.motors[m.pin] = NullMotor(m.name)
        return self.motors[m.pin]

    def level(self, gpio):
        pin = next(p for p, g in GPIO.items() if g == gpio)
        return self.stuck if self.stuck is not None else bool(self.motors[pin].on)

    def all_off_and_released(self):
        return all(m.closed and m.on is False for m in self.motors.values())


def _run(rig, args=(), answers=None):
    it = iter(answers or [])
    ask = (lambda q: next(it)) if answers is not None else None
    return H.main(list(args) + ([] if answers is not None else ["--yes"]), motor_factory=rig.factory, level_reader=rig.level,
                  ask=ask, clock=FakeClock(0.0))


def test_passes_when_every_motor_is_driven_high_and_felt(capsys):
    rig = Rig()
    assert _run(rig, answers=["r", "y", "y", "y"]) == 0                      # "r" repeats the first motor
    out = capsys.readouterr().out
    assert "100 % duty" in out and out.count("10/10 high while on, 10/10 low while off  ok") == 3 and "PASS" in out
    assert rig.motors["D12"].n_writes > rig.motors["D13"].n_writes and rig.all_off_and_released()


def test_pin_not_held_high_fails(capsys):
    rig = Rig(stuck=False)
    assert _run(rig) == 1
    out = capsys.readouterr().out
    assert "0/10 high while on" in out and "duty below 100 %" in out and out.rstrip().endswith("FAIL")


def test_motor_not_felt_fails(capsys):
    rig = Rig()
    assert _run(rig, answers=["y", "n", "y"]) == 1
    assert "right   D13   RIGHT  ok      ok      NO" in capsys.readouterr().out


def test_busy_pin_is_reported_with_a_hint_and_the_rest_still_tested(capsys):
    rig = Rig(busy={"D12"})
    assert _run(rig) == 1
    out = capsys.readouterr().out
    assert "GPIO busy" in out and "systemctl stop goldenfleece" in out and "skipped" in out
    assert rig.motors["D13"].n_writes > 0 and rig.all_off_and_released()


def test_pins_override_and_its_check(capsys):
    rig = Rig()
    assert _run(rig, ["--pins", "D20,D21"]) == 0 and set(rig.motors) == {"D20", "D21"}
    assert _run(Rig(), ["--pins", "D20"]) == 2
    assert "2 motors" in capsys.readouterr().err


def test_patterns_play_through_the_pipeline_renderer(capsys):
    rig = Rig()
    assert _run(rig, ["--patterns", "--pattern-s", "2"]) == 0
    out = capsys.readouterr().out
    for name in ("advisory", "warning", "alert", "degraded", "offline"):
        assert f"  {name}" in out
    assert "warnings offline: a fault, never a threat" in out and rig.all_off_and_released()


def test_off_switches_every_motor_off(capsys):
    rig = Rig()
    assert H.main(["--off"], motor_factory=rig.factory) == 0
    assert rig.all_off_and_released() and set(rig.motors) == {"D12", "D13"}
    assert H.main(["--off"], motor_factory=Rig(busy={"D13"}).factory) == 1


def test_interrupt_leaves_every_motor_off(capsys):
    rig = Rig()

    def ask(q):
        raise KeyboardInterrupt
    assert H.main([], motor_factory=rig.factory, level_reader=rig.level, ask=ask, clock=FakeClock(0.0)) == 130
    assert rig.all_off_and_released() and "switching every motor off" in capsys.readouterr().out


@pytest.mark.parametrize("stdout, level", [("12: op dh pd | hi // GPIO12 = output\n", True),
                                           ("12: op dl pd | lo // GPIO12 = output\n", False),
                                           ("unknown gpio\n", None)])
def test_pinctrl_level_parse(monkeypatch, stdout, level):
    monkeypatch.setattr(subprocess, "run", lambda *a, **k: subprocess.CompletedProcess(a, 0, stdout=stdout, stderr=""))
    assert H.pinctrl_level(12) is level

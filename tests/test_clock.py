from goldenfleece.clock import FakeClock, MonotonicClock
import pytest


def test_fake_clock_advances_and_sleeps():
    c = FakeClock(10.0)
    assert c.now() == 10.0
    c.advance(0.5)
    assert c.now() == 10.5
    c.sleep(0.25)
    assert c.now() == 10.75
    c.set(20.0)
    assert c.now() == 20.0
    with pytest.raises(ValueError):
        c.advance(-1)
    with pytest.raises(ValueError):
        c.set(1.0)


def test_monotonic_clock_is_monotone():
    c = MonotonicClock()
    a = c.now()
    c.sleep(0.001)
    b = c.now()
    assert b >= a + 0.0009

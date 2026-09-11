import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from goldenfleece.clock import FakeClock  # noqa: E402
from goldenfleece.config import load_config  # noqa: E402


@pytest.fixture(scope="session")
def cfg():
    return load_config(ROOT / "config")


@pytest.fixture
def clock():
    return FakeClock(100.0)


@pytest.fixture(scope="session")
def repo_root():
    return ROOT

"""Invariant 9: every injected health reason reaches detection within its documented worst-case time
(docs/latency_budget.md), measured in the simulator."""
import pytest

from tools.sim.run import run_scenario
from tools.sim.scenarios import BY_NAME

DOCUMENTED_MAX_S = {"RADAR_SILENT": 0.205, "IMU_FAULT": 0.16, "RADAR_GAPS": 1.06, "RADAR_POSSIBLY_BLOCKED": 5.06}


@pytest.mark.parametrize("scenario,bit", [("radar_silence", "RADAR_SILENT"), ("imu_fault", "IMU_FAULT"),
                                          ("radar_frame_gaps", "RADAR_GAPS"), ("radar_blocked_mid_ride", "RADAR_POSSIBLY_BLOCKED")])
def test_onset_to_detection_within_documented_bound(cfg, scenario, bit):
    m = run_scenario(BY_NAME[scenario](), cfg)
    lat = m.health_onset_latency_s.get(bit)
    assert lat is not None, f"{bit} never detected"
    assert lat <= DOCUMENTED_MAX_S[bit], f"{bit}: {lat:.3f} s > documented {DOCUMENTED_MAX_S[bit]} s"

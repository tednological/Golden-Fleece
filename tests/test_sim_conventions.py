"""The simulator must reproduce the task's sign conventions before it can gate any layer."""
import math

import numpy as np

from tools.sim.imu_model import ImuModel, ImuModelParams
from tools.sim.kinematics import RiderKinematics, RiderProfile, ShoulderCheck
from tools.sim.radar_model import RadarModel, RadarModelParams
from tools.sim.road import Road, RoadSegment
from tools.sim.world import BIKE_POINTS, Scatterer, Vehicle, World


def _quiet_params(**kw):
    return RadarModelParams(angle_noise_deg=0.0, range_noise_m=0.0, false_alarm_rate_per_frame=0.0, usb_jitter_s=0.0, **kw)


def test_pole_behind_recedes_at_rider_speed():
    world = World(Road(), scatterers=[Scatterer(s=-10.0, lateral_m=0.0, z=1.2, rcs_m2=5.0)])
    rider = RiderKinematics(world.road, RiderProfile(speed_keypoints=((0, 5.0),)))
    rm = RadarModel(world, rider, _quiet_params())
    frame, truth = rm.frame(0.0)
    assert frame is not None and len(frame.targets) == 1
    t = frame.targets[0]
    assert t.speed_raw > 0                                  # receding = positive raw speed
    assert abs(t.speed_raw / 100 / 3.6 - 5.0) < 0.15        # one bin = 0.22 m/s
    assert abs(t.angle_raw) <= 100
    assert abs(t.distance_cm / 100 - 10.0) < 0.31


def test_pole_at_30deg_right_gives_negative_wire_angle_and_4p33():
    # rider's RIGHT is negative lateral in sim_world; az_radar = +30 deg
    world = World(Road(), scatterers=[Scatterer(s=-10 * math.cos(math.radians(30)), lateral_m=-10 * math.sin(math.radians(30)), z=1.2, rcs_m2=5.0)])
    rider = RiderKinematics(world.road, RiderProfile(speed_keypoints=((0, 5.0),)))
    rm = RadarModel(world, rider, _quiet_params())
    frame, truth = rm.frame(0.0)
    t = frame.targets[0]
    assert -3100 <= t.angle_raw <= -2900                     # rider's right => NEGATIVE K-LD7 angle
    assert abs(t.speed_raw / 100 / 3.6 - 4.330) < 0.15


def test_car_on_riders_left_gives_positive_wire_angle_and_negative_speed():
    world = World(Road(), vehicles=[Vehicle("car", s0=-20.0, speed_mps=15.0, lateral_m=+3.0)])
    rider = RiderKinematics(world.road, RiderProfile(speed_keypoints=((0, 5.0),)))
    rm = RadarModel(world, rider, _quiet_params())
    frame, truth = rm.frame(0.0)
    strongest = max(frame.targets, key=lambda x: x.magnitude_raw)
    assert strongest.angle_raw > 0          # LEFT => positive K-LD7 angle (=> negative y after decode)
    assert strongest.speed_raw < 0          # approaching => negative raw speed
    assert abs(strongest.speed_raw / 100 / 3.6 + 10.0) < 0.5
    v = truth.vehicles[0]
    assert v.side == "LEFT" and v.t_arrival is not None and abs(v.t_arrival - 2.0) < 0.05
    assert v.az < 0                          # rider's LEFT is -Y


def test_accelerometer_reads_plus_g_on_up_axis_at_rest():
    world = World(Road())
    rider = RiderKinematics(world.road, RiderProfile(speed_keypoints=((0, 0.0),), vibration_rms_mps2=0.0))
    imu = ImuModel(rider, ImuModelParams(gyro_noise_rad_s=0, accel_noise_mps2=0, gyro_bias_rad_s=(0, 0, 0), accel_bias_mps2=(0, 0, 0), timestamp_jitter_s=0))
    samples = imu.samples_until(0.05)
    acc = [s for s in samples if s.kind.value == "a"]
    assert acc and np.allclose(acc[0].values, [0, 0, 9.80665], atol=1e-6)


def test_shoulder_check_left_gives_positive_gyro_z():
    world = World(Road())
    prof = RiderProfile(speed_keypoints=((0, 5.0),), shoulder_checks=[ShoulderCheck(0.0, 1.0, math.radians(45))], vibration_rms_mps2=0.0)
    rider = RiderKinematics(world.road, prof)
    w = rider.body_rates(0.25)     # torso yawing LEFT (counterclockwise from above) at peak rate
    assert w[2] > 1.0 and abs(w[0]) < 1e-3 and abs(w[1]) < 1e-3


def test_coordinated_turn_specific_force_is_along_body_up():
    road = Road([RoadSegment(1000.0, 0.05)])
    world = World(road)
    rider = RiderKinematics(road, RiderProfile(speed_keypoints=((0, 10.0),), vibration_rms_mps2=0.0))
    st = rider.state(1.0)
    assert abs(st.lean - math.atan2(100 * 0.05, 9.80665)) < 1e-9
    from tools.sim.kinematics import G
    f_r = st.R_r_w @ (st.a_w - np.array([0, 0, -G]))
    assert abs(f_r[1]) < 1e-6 and abs(f_r[0]) < 1e-6        # the degeneracy: naive leveling sees no roll
    assert f_r[2] > G


def test_doppler_bin_merging_reduces_target_count():
    rng = np.random.default_rng(3)
    sc = World.roadside_clutter(rng, -40, 5, density_per_m=1.0)
    world = World(Road(), scatterers=sc)
    rider = RiderKinematics(world.road, RiderProfile(speed_keypoints=((0, 8.0),)))
    rm = RadarModel(world, rider, _quiet_params())
    frame, truth = rm.frame(0.0)
    assert truth.n_above_threshold > truth.n_bins_after_merge      # merging happened
    assert truth.n_clutter_bins <= 10                                # 8 m/s -> at most ~9-10 clutter bins
    assert len(frame.targets) <= 12

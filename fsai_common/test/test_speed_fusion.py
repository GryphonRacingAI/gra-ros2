import math

from fsai_common.speed_fusion import SpeedFusion

DT = 0.01


def test_follows_the_wheels_when_they_agree():
    f = SpeedFusion()
    v = 0.0
    for _ in range(500):                       # 5 s at 2 m/s^2
        v += 2.0 * DT
        f.update(v, 2.0, DT)
    assert abs(f.v - v) < 0.05 and not f.slipping


def test_imu_gap_takes_the_wheel_speed():
    # the autocross case: 4.8 m/s, one braking sample (-4 m/s^2) arriving after a 0.96 s gap
    f = SpeedFusion()
    f.v = 4.8
    assert f.update(4.8, -4.0, 0.96) == 'gap'
    assert f.v == 4.8 and not f.slipping


def test_short_spin_up_is_not_believed():
    f = SpeedFusion()
    f.v = 5.0
    assert f.update(9.0, 0.0, DT) == 'slip'    # wheels jump +4 m/s, the car does not accelerate
    for _ in range(79):
        f.update(9.0, 0.0, DT)
    assert f.slipping and f.v < 5.7            # 0.8 s later: still close to the true 5 m/s


def slide(v0, decel):
    """Locked wheels from v0 at a constant deceleration: (distance by the fusion, true distance)."""
    f = SpeedFusion()
    f.v = v0
    v_true, d, d_true = v0, 0.0, 0.0
    while v_true > 0.0:
        f.update(0.0, -decel, DT)
        v_true = max(0.0, v_true - decel * DT)
        d += f.v * DT
        d_true += v_true * DT
    return d, d_true


def test_locked_wheel_slide_distance():
    for g in (0.6, 1.0):
        d, d_true = slide(10.0, g * 9.81)
        assert abs(d - d_true) / d_true < 0.03, (g, d, d_true)


def test_locked_then_stopped_goes_to_zero_despite_accelerometer_bias():
    f = SpeedFusion()
    f.v = 3.0
    assert f.update(0.0, -6.0, DT) == 'locked'
    for _ in range(40):
        f.update(0.0, -6.0, DT)                # slide: the car stops in reality after 0.5 s
    for _ in range(1000):
        f.update(0.0, 0.1, DT)                 # stopped, the accelerometer has a +0.1 m/s^2 bias
    assert f.v == 0.0 and not f.slipping


def test_standing_stays_zero():
    f = SpeedFusion()
    for _ in range(100):
        f.update(0.0, 0.3, DT)
    assert f.v == 0.0 and math.isfinite(f.v)

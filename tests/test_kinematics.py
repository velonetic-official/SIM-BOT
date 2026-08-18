"""기구학 / 오도메트리 수식 검증 (하드웨어·CAN 불필요)."""
import math

import pytest

from candrive.config import HardwareConfig
from candrive.kinematics import DiffDriveKinematics, Odometry, Twist


@pytest.fixture
def hw():
    return HardwareConfig.load()


def test_config_derived_values(hw):
    assert hw.counts_per_wheel_rev == pytest.approx(
        hw.counts_per_motor_rev * hw.gear_ratio)
    # 휠 1회전 = 원주만큼 이동
    assert hw.meters_per_count * hw.counts_per_wheel_rev == pytest.approx(
        2 * math.pi * hw.wheel_radius_m)


def test_straight_line_no_rotation(hw):
    kin = DiffDriveKinematics(hw)
    l, r = kin.twist_to_wheel_rpm(Twist(0.4, 0.0))
    assert l == pytest.approx(r)
    # 0.4 m/s → 휠 rpm = 0.4 / (2πr) * 60
    expect = 0.4 / (2 * math.pi * hw.wheel_radius_m) * 60.0
    assert l == pytest.approx(expect, rel=1e-6)


def test_pure_rotation_is_antisymmetric(hw):
    kin = DiffDriveKinematics(hw)
    l, r = kin.twist_to_wheel_rpm(Twist(0.0, 1.0))
    assert l == pytest.approx(-r)
    # ω=1 rad/s → 휠 선속도 = ω * b/2
    v = 1.0 * hw.wheel_separation_m / 2.0
    expect = v / (2 * math.pi * hw.wheel_radius_m) * 60.0
    assert r == pytest.approx(expect, rel=1e-6)


def test_forward_inverse_roundtrip(hw):
    kin = DiffDriveKinematics(hw)
    for v, w in [(0.2, 0.0), (0.0, 0.7), (0.35, 0.4), (-0.25, -0.3)]:
        lm, rm = kin.twist_to_wheel_mps(Twist(v, w))
        back = kin.wheel_mps_to_twist(lm, rm)
        assert back.linear == pytest.approx(v, abs=1e-9)
        assert back.angular == pytest.approx(w, abs=1e-9)


def test_limits_are_enforced(hw):
    kin = DiffDriveKinematics(hw)
    l, r = kin.twist_to_wheel_rpm(Twist(99.0, 0.0))
    assert abs(l) <= hw.max_wheel_rpm + 1e-6
    assert abs(r) <= hw.max_wheel_rpm + 1e-6


def _drive_counts(hw, odom, wheel_mps_l, wheel_mps_r, duration, dt=0.02):
    """주어진 휠 속도로 duration 동안 굴렸을 때의 카운트를 만들어 적분."""
    cl = cr = 0.0
    circ_l = 2 * math.pi * hw.wheel_radius("left")
    circ_r = 2 * math.pi * hw.wheel_radius("right")
    odom.update(0, 0, dt)
    remaining = duration
    while remaining > 1e-12:
        h = min(dt, remaining)          # 마지막 스텝의 나머지까지 정확히 적분
        remaining -= h
        cl += wheel_mps_l / circ_l * hw.counts_per_wheel_rev * h
        cr += wheel_mps_r / circ_r * hw.counts_per_wheel_rev * h
        odom.update(int(round(cl)), int(round(cr)), h)
    return odom.pose


def test_odometry_straight(hw):
    odom = Odometry(hw)
    pose = _drive_counts(hw, odom, 0.5, 0.5, 4.0)
    assert pose.x == pytest.approx(2.0, abs=0.01)
    assert pose.y == pytest.approx(0.0, abs=1e-3)
    assert pose.theta == pytest.approx(0.0, abs=1e-3)


def test_odometry_in_place_rotation(hw):
    odom = Odometry(hw)
    v = 1.0 * hw.wheel_separation_m / 2.0        # ω = 1 rad/s
    _drive_counts(hw, odom, -v, v, math.pi / 2)  # 90도
    assert odom.pose.theta == pytest.approx(math.pi / 2, abs=0.01)
    assert odom.pose.x == pytest.approx(0.0, abs=0.005)
    assert odom.pose.y == pytest.approx(0.0, abs=0.005)


def test_odometry_arc_matches_analytic_circle(hw):
    """v=0.4, ω=0.4 → 반지름 1m 원. 1/4바퀴 후 (1,1) 근처."""
    odom = Odometry(hw)
    b = hw.wheel_separation_m
    v, w = 0.4, 0.4
    lm, rm = v - w * b / 2, v + w * b / 2
    _drive_counts(hw, odom, lm, rm, (math.pi / 2) / w)
    r = v / w
    assert odom.pose.x == pytest.approx(r, abs=0.02)
    assert odom.pose.y == pytest.approx(r, abs=0.02)
    assert odom.pose.theta == pytest.approx(math.pi / 2, abs=0.02)


def test_encoder_wraparound_is_handled(hw):
    """i32 경계를 넘어가도 증분이 튀지 않아야 한다."""
    odom = Odometry(hw)
    big = 2**31 - 500
    odom.update(big, big, 0.02)
    wrapped = -(2**31) + 500          # +1000 카운트 진행
    odom.update(wrapped, wrapped, 0.02)
    expect = 1000 * hw.meters_per_count
    assert odom.pose.x == pytest.approx(expect, rel=1e-6)


def test_slew_limiter_respects_accel(hw):
    from candrive.kinematics import SlewLimiter
    sl = SlewLimiter(hw)
    out = sl.step(Twist(1.0, 0.0), 0.02)
    assert out.linear == pytest.approx(hw.max_linear_accel * 0.02)

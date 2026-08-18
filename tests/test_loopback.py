"""가상 CAN 버스 엔드투엔드 테스트 — 마스터 ↔ 가상 드라이브."""
import math
import time

import pytest

from candrive.bus import CanLink
from candrive.config import Settings
from candrive.runner import DriveController
from candrive.sim.node import VirtualDrive


@pytest.fixture
def stack():
    s = Settings.load(backend="virtual")
    sim_link = CanLink(s.bus, name="sim")
    sim = VirtualDrive(sim_link, s.hw, s.profile)
    sim.start()
    ctl = DriveController(s, verbose=False)
    ctl.bringup()
    yield s, sim, ctl
    ctl.shutdown()
    sim.stop()
    sim_link.close()


def _spin(ctl, v, w, seconds):
    period = ctl.s.hw.control_period_s
    end = time.monotonic() + seconds
    last = None
    while time.monotonic() < end:
        t0 = time.monotonic()
        ctl.set_cmd_vel(v, w)
        last = ctl.tick(t0)
        time.sleep(max(0.0, period - (time.monotonic() - t0)))
    return last


def test_bringup_reaches_operation_enabled(stack):
    _, _, ctl = stack
    for ax in (1, 2):
        assert ctl.drive.telemetry.axes[ax].state == "operation_enabled"


def test_heartbeat_is_alive(stack):
    _, _, ctl = stack
    time.sleep(0.4)
    assert ctl.drive.telemetry.online(), "드라이브 하트비트 수신 실패"


def test_sdo_read_after_write(stack):
    _, _, ctl = stack
    ctl.drive.sdo_write("profile_accel", 1, 12345)
    assert ctl.drive.sdo_read("profile_accel", 1) == 12345


def test_sdo_abort_on_unknown_object(stack):
    _, _, ctl = stack
    with pytest.raises(RuntimeError, match="abort"):
        ctl.drive.link.send(
            ctl.s.profile.cob["sdo_rx_base"] + ctl.s.hw.node_id,
            b"\x40\x00\x99\x00\x00\x00\x00\x00")
        ctl.drive._await_sdo(0x9900, 0, 0.5, "unknown")


def test_forward_command_produces_forward_odometry(stack):
    _, _, ctl = stack
    smp = _spin(ctl, 0.4, 0.0, 3.0)
    assert smp.x > 0.5, f"전진 지령인데 x={smp.x:.3f}"
    assert abs(smp.y) < 0.15
    assert abs(smp.theta_deg) < 12


def test_left_turn_increases_theta(stack):
    _, _, ctl = stack
    smp = _spin(ctl, 0.0, 0.6, 2.0)
    assert smp.theta_deg > 20, f"좌회전인데 θ={smp.theta_deg:.1f}°"


def test_right_turn_decreases_theta(stack):
    _, _, ctl = stack
    smp = _spin(ctl, 0.0, -0.6, 2.0)
    assert smp.theta_deg < -20


def test_feedback_tracks_command_in_steady_state(stack):
    _, _, ctl = stack
    smp = _spin(ctl, 0.35, 0.0, 3.0)
    assert smp.fb_left_rpm == pytest.approx(smp.cmd_left_rpm, rel=0.05)
    assert smp.fb_right_rpm == pytest.approx(smp.cmd_right_rpm, rel=0.05)


def test_watchdog_stops_when_commands_stop(stack):
    _, _, ctl = stack
    _spin(ctl, 0.4, 0.0, 1.5)
    # 지령을 끊고 계속 tick 만 돌린다
    period = ctl.s.hw.control_period_s
    for _ in range(int(2.0 / period)):
        smp = ctl.tick()
        time.sleep(period)
    assert ctl.watchdog_tripped
    assert abs(smp.fb_left_rpm) < 1.0 and abs(smp.fb_right_rpm) < 1.0


def test_odometry_matches_simulator_ground_truth(stack):
    """마스터가 CAN 으로만 본 오도메트리 vs 시뮬 내부 실제 카운트."""
    s, sim, ctl = stack
    _spin(ctl, 0.4, 0.25, 4.0)
    _spin(ctl, 0.0, 0.0, 0.6)

    truth = sim.truth()
    hw = s.hw
    # 시뮬 카운트를 좌/우 휠 이동거리로 환산 (부호반전 반영)
    dist = {}
    for side in ("left", "right"):
        spec = hw.spec(side)
        c = truth["axes"][spec.axis]["counts"] * (-1 if spec.invert_feedback else 1)
        circ = 2 * math.pi * hw.wheel_radius(side)
        dist[side] = c / hw.counts_per_wheel_rev * circ

    expect_theta = (dist["right"] - dist["left"]) / hw.wheel_separation_m
    got = math.radians(ctl.odom.pose.theta * 180 / math.pi)  # = pose.theta
    assert ctl.odom.pose.theta == pytest.approx(expect_theta, abs=0.05), (
        f"오도 θ={ctl.odom.pose.theta:.3f} vs 시뮬 실제 {expect_theta:.3f}")

    expect_dist = (dist["left"] + dist["right"]) / 2.0
    assert ctl.odom.total_distance == pytest.approx(abs(expect_dist), rel=0.05)


def test_can_traffic_uses_expected_cob_ids(stack):
    s, _, ctl = stack
    _spin(ctl, 0.2, 0.0, 0.5)
    ids = {int(f["id"], 16) for f in ctl.link.recent(500)}
    nid = s.hw.node_id
    for p in s.profile.rx_pdos:
        assert p.cob_base + nid in ids, f"RPDO 0x{p.cob_base+nid:03X} 미송신"
    for p in s.profile.tx_pdos:
        assert p.cob_base + nid in ids, f"TPDO 0x{p.cob_base+nid:03X} 미수신"
    assert s.profile.cob["heartbeat_base"] + nid in ids


def test_all_frames_are_at_most_8_bytes(stack):
    _, _, ctl = stack
    _spin(ctl, 0.3, 0.3, 0.5)
    for f in ctl.link.recent(500):
        assert f["dlc"] <= 8, f"8바이트 초과 프레임: {f}"

"""상위 제어 루프 + 데모 러너.

DriveController: cmd_vel(Twist) 를 받아 슬루제한 → 역기구학 → CAN RPDO,
                 TPDO 피드백 → 오도메트리 적분. ROS2 노드와 CLI 가 공유한다.
DemoRunner:      하드웨어 없이 시뮬 드라이브까지 띄워서 시나리오 주행.
"""
from __future__ import annotations

import csv
import json
import math
import os
import time
from dataclasses import asdict, dataclass, field
from typing import Callable, Dict, List, Optional

from .bus import CanLink
from .config import BusConfig, HardwareConfig, ProfileConfig, Settings
from .drive import DualAxisDrive
from .kinematics import DiffDriveKinematics, Odometry, Pose2D, SlewLimiter, Twist


# --------------------------------------------------------------------------- #
@dataclass
class Sample:
    t: float
    cmd_v: float
    cmd_w: float
    slew_v: float
    slew_w: float
    cmd_left_rpm: float
    cmd_right_rpm: float
    fb_left_rpm: float
    fb_right_rpm: float
    left_counts: int
    right_counts: int
    x: float
    y: float
    theta_deg: float
    odom_v: float
    odom_w: float
    odom_v_raw: float
    odom_w_raw: float
    axis1_state: str
    axis2_state: str


# --------------------------------------------------------------------------- #
class DriveController:
    """CAN 드라이브 + 오도메트리를 묶은 상위 제어기."""

    def __init__(self, settings: Settings, link: Optional[CanLink] = None,
                 verbose: bool = True):
        self.s = settings
        self.hw = settings.hw
        self.link = link or CanLink(settings.bus, name="master")
        self.drive = DualAxisDrive(self.link, self.hw, settings.profile)
        self.kin = DiffDriveKinematics(self.hw)
        self.slew = SlewLimiter(self.hw)
        self.odom = Odometry(self.hw, velocity_filter_hz=self.hw.odom_lpf_hz)
        self.verbose = verbose

        self.cmd = Twist()
        self.last_cmd_time = time.monotonic()
        self._last_tick = time.monotonic()
        self.applied = Twist()
        self.cmd_rpm = (0.0, 0.0)
        self.watchdog_tripped = False

    # ------------------------------------------------------------------ 기동
    def bringup(self) -> None:
        if self.verbose:
            print(self.hw.summary())
            print(f"[bus] {self.link.describe()}")
        self.drive.bringup(verbose=self.verbose)
        self.drive.enable(verbose=self.verbose)

    # --------------------------------------------------------------- 지령입력
    def set_cmd_vel(self, linear: float, angular: float) -> None:
        self.cmd = Twist(linear, angular)
        self.last_cmd_time = time.monotonic()
        self.watchdog_tripped = False

    # ------------------------------------------------------------------ 주기
    def tick(self, now: Optional[float] = None) -> Sample:
        now = now or time.monotonic()
        dt = max(1e-4, now - self._last_tick)
        self._last_tick = now

        # watchdog: 지령이 끊기면 0으로
        target = self.cmd
        if (now - self.last_cmd_time) > self.hw.cmd_timeout_s:
            target = Twist(0.0, 0.0)
            self.watchdog_tripped = True

        self.applied = self.slew.step(target, dt)
        l_rpm, r_rpm = self.kin.twist_to_wheel_rpm(self.applied)
        self.cmd_rpm = (l_rpm, r_rpm)
        self.drive.set_wheel_rpm(l_rpm, r_rpm)
        self.drive.tick()

        fb = self.drive.wheel_feedback()
        self.odom.update(int(fb["left"]["counts"]), int(fb["right"]["counts"]), dt)

        return Sample(
            t=now, cmd_v=self.cmd.linear, cmd_w=self.cmd.angular,
            slew_v=self.applied.linear, slew_w=self.applied.angular,
            cmd_left_rpm=l_rpm, cmd_right_rpm=r_rpm,
            fb_left_rpm=fb["left"]["wheel_rpm"], fb_right_rpm=fb["right"]["wheel_rpm"],
            left_counts=int(fb["left"]["counts"]), right_counts=int(fb["right"]["counts"]),
            x=self.odom.pose.x, y=self.odom.pose.y,
            theta_deg=math.degrees(self.odom.pose.theta),
            odom_v=self.odom.twist.linear, odom_w=self.odom.twist.angular,
            odom_v_raw=self.odom.twist_raw.linear,
            odom_w_raw=self.odom.twist_raw.angular,
            axis1_state=self.drive.telemetry.axes[1].state,
            axis2_state=self.drive.telemetry.axes[2].state,
        )

    def shutdown(self) -> None:
        self.drive.close()
        self.link.close()


# --------------------------------------------------------------------------- #
# 데모 시나리오
# --------------------------------------------------------------------------- #
@dataclass
class Segment:
    name: str
    duration: float
    linear: float
    angular: float


DEFAULT_SCENARIO: List[Segment] = [
    Segment("정지 대기",        1.0,  0.00,  0.00),
    Segment("전진 0.4 m/s",     4.0,  0.40,  0.00),
    Segment("좌선회 (호)",      4.0,  0.35,  0.50),
    Segment("직진",             3.0,  0.50,  0.00),
    Segment("우선회 (호)",      4.0,  0.35, -0.50),
    Segment("제자리 좌회전",    3.0,  0.00,  0.80),
    Segment("후진",             3.0, -0.30,  0.00),
    Segment("정지",             2.0,  0.00,  0.00),
]

SQUARE_SCENARIO: List[Segment] = (
    [Segment("직진 1m", 2.5, 0.40, 0.0),
     Segment("90도 좌회전", 2.094, 0.0, 0.75)] * 4
)


# --------------------------------------------------------------------------- #
class DemoRunner:
    """하드웨어 0개로 전체 스택 구동."""

    def __init__(self, backend: str = "virtual", verbose: bool = True,
                 with_simulator: bool = True, settings: Optional[Settings] = None):
        self.s = settings or Settings.load(backend=backend)
        self.verbose = verbose
        self.with_simulator = with_simulator
        self.sim = None
        self.sim_link: Optional[CanLink] = None
        self.samples: List[Sample] = []

    def __enter__(self) -> "DemoRunner":
        if self.with_simulator:
            from .sim.node import VirtualDrive
            self.sim_link = CanLink(self.s.bus, name="drive-sim")
            self.sim = VirtualDrive(self.sim_link, self.s.hw, self.s.profile)
            self.sim.start()
            if self.verbose:
                print(f"[sim] 가상 2ELD2-CAN 드라이브 기동 "
                      f"(node {self.s.hw.node_id}) — 실물 하드웨어 없음")
        self.ctl = DriveController(self.s, verbose=self.verbose)
        self.ctl.bringup()
        return self

    def __exit__(self, *exc):
        self.ctl.shutdown()
        if self.sim:
            self.sim.stop()
        if self.sim_link:
            self.sim_link.close()

    # ------------------------------------------------------------------ 실행
    def run(self, scenario: Optional[List[Segment]] = None,
            on_sample: Optional[Callable[[Sample, str], None]] = None
            ) -> List[Sample]:
        scenario = scenario or DEFAULT_SCENARIO
        period = self.s.hw.control_period_s
        t_start = time.monotonic()
        if self.verbose:
            print("\n" + "=" * 78)
            print(f"{'t[s]':>6} {'구간':<16} {'v':>6} {'w':>6} "
                  f"{'L rpm':>7} {'R rpm':>7} {'x':>7} {'y':>7} {'θ°':>7}")
            print("=" * 78)

        for seg in scenario:
            self.ctl.set_cmd_vel(seg.linear, seg.angular)
            seg_end = time.monotonic() + seg.duration
            last_print = 0.0
            while time.monotonic() < seg_end:
                loop_start = time.monotonic()
                self.ctl.set_cmd_vel(seg.linear, seg.angular)   # watchdog 갱신
                smp = self.ctl.tick(loop_start)
                smp.t = loop_start - t_start
                self.samples.append(smp)
                if on_sample:
                    on_sample(smp, seg.name)
                if self.verbose and smp.t - last_print >= 0.5:
                    last_print = smp.t
                    print(f"{smp.t:6.2f} {seg.name:<16} {smp.slew_v:6.2f} "
                          f"{smp.slew_w:6.2f} {smp.fb_left_rpm:7.1f} "
                          f"{smp.fb_right_rpm:7.1f} {smp.x:7.3f} {smp.y:7.3f} "
                          f"{smp.theta_deg:7.1f}")
                sleep = period - (time.monotonic() - loop_start)
                if sleep > 0:
                    time.sleep(sleep)

        # 정지 확인
        self.ctl.set_cmd_vel(0.0, 0.0)
        for _ in range(int(1.5 / period)):
            t0 = time.monotonic()
            self.ctl.set_cmd_vel(0.0, 0.0)
            smp = self.ctl.tick(t0)
            smp.t = t0 - t_start
            self.samples.append(smp)
            if on_sample:
                on_sample(smp, "정지 확인")
            time.sleep(max(0.0, period - (time.monotonic() - t0)))
        return self.samples

    # ------------------------------------------------------------------ 출력
    def save_csv(self, path: str) -> str:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=list(asdict(self.samples[0])))
            w.writeheader()
            for s in self.samples:
                w.writerow(asdict(s))
        return path

    def save_trace(self, path: str) -> str:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(self.ctl.link.recent(self.s.bus.trace_max), f,
                      ensure_ascii=False, indent=1)
        return path

    def report(self) -> dict:
        if not self.samples:
            return {}
        last = self.samples[-1]
        truth = self.sim.truth() if self.sim else None
        rep = {
            "samples": len(self.samples),
            "duration_s": round(last.t, 2),
            "final_pose": {"x": round(last.x, 4), "y": round(last.y, 4),
                           "theta_deg": round(last.theta_deg, 2)},
            "path_length_m": round(self.ctl.odom.total_distance, 3),
            "can_tx_frames": self.ctl.link.tx_count,
            "can_rx_frames": self.ctl.link.rx_count,
            "bus": self.ctl.link.describe(),
        }
        if truth:
            rep["sim_truth_counts"] = {
                f"axis{ax}": round(v["counts"], 1)
                for ax, v in truth["axes"].items()}
        return rep

"""차동구동 기구학 + 오도메트리.

- 역기구학: (v, ω) → 좌/우 휠 선속도 → 휠 rpm
- 정기구학: 엔코더 카운트 증분 → ΔX, ΔY, Δθ (2차 정확도 원호적분)
- 슬루레이트 리미터로 급지령 완충
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple

from .config import HardwareConfig


# --------------------------------------------------------------------------- #
@dataclass
class Twist:
    linear: float = 0.0      # m/s
    angular: float = 0.0     # rad/s


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0

    def as_tuple(self) -> Tuple[float, float, float]:
        return self.x, self.y, self.theta


def normalize_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


# --------------------------------------------------------------------------- #
class DiffDriveKinematics:
    def __init__(self, hw: HardwareConfig):
        self.hw = hw

    # ---- 역기구학 --------------------------------------------------------
    def twist_to_wheel_mps(self, tw: Twist) -> Tuple[float, float]:
        b = self.hw.wheel_separation_m
        left = tw.linear - tw.angular * b / 2.0
        right = tw.linear + tw.angular * b / 2.0
        return left, right

    def twist_to_wheel_rpm(self, tw: Twist) -> Tuple[float, float]:
        """지령 twist → (좌 휠 rpm, 우 휠 rpm). 한계 초과 시 비율 유지 스케일다운."""
        tw = self.clamp_twist(tw)
        l_mps, r_mps = self.twist_to_wheel_mps(tw)
        max_mps = self.hw.wheel_mps_from_motor_rpm(self.hw.max_motor_rpm, "right")
        peak = max(abs(l_mps), abs(r_mps))
        if peak > max_mps > 0:
            k = max_mps / peak
            l_mps, r_mps = l_mps * k, r_mps * k
        l_rpm = self.hw.motor_rpm_from_wheel_mps(l_mps, "left") / self.hw.gear_ratio
        r_rpm = self.hw.motor_rpm_from_wheel_mps(r_mps, "right") / self.hw.gear_ratio
        return l_rpm, r_rpm

    def clamp_twist(self, tw: Twist) -> Twist:
        return Twist(
            max(-self.hw.max_linear_mps, min(self.hw.max_linear_mps, tw.linear)),
            max(-self.hw.max_angular_rps, min(self.hw.max_angular_rps, tw.angular)),
        )

    # ---- 정기구학 --------------------------------------------------------
    def wheel_mps_to_twist(self, left_mps: float, right_mps: float) -> Twist:
        b = self.hw.wheel_separation_m
        return Twist((left_mps + right_mps) / 2.0, (right_mps - left_mps) / b)


# --------------------------------------------------------------------------- #
class SlewLimiter:
    """가속도 제한 (지령 급변 완충)."""

    def __init__(self, hw: HardwareConfig):
        self.hw = hw
        self.cur = Twist()

    def step(self, target: Twist, dt: float) -> Twist:
        dv = self.hw.max_linear_accel * dt
        dw = self.hw.max_angular_accel * dt
        self.cur.linear += max(-dv, min(dv, target.linear - self.cur.linear))
        self.cur.angular += max(-dw, min(dw, target.angular - self.cur.angular))
        return Twist(self.cur.linear, self.cur.angular)

    def reset(self) -> None:
        self.cur = Twist()


# --------------------------------------------------------------------------- #
class Odometry:
    """엔코더 카운트 기반 오도메트리 적산기.

    카운트는 i32 랩어라운드를 가정하고 증분을 안전하게 계산한다.
    """

    INT32_SPAN = 1 << 32

    def __init__(self, hw: HardwareConfig, velocity_filter_hz: float = 5.0):
        self.hw = hw
        self.kin = DiffDriveKinematics(hw)
        self.pose = Pose2D()
        self.twist = Twist()          # 저역통과된 속도 (발행/표시용)
        self.twist_raw = Twist()      # 카운트 차분 원시값
        self.fc = velocity_filter_hz  # 속도 추정 LPF 차단주파수
        self._prev = {"left": None, "right": None}
        self.total_distance = 0.0

    def _filter_twist(self, raw: Twist, dt: float) -> None:
        """엔코더 차분 미분은 양자화 잡음이 크므로 1차 LPF 를 건다."""
        self.twist_raw = raw
        if self.fc <= 0:
            self.twist = raw
            return
        a = dt / (dt + 1.0 / (2.0 * math.pi * self.fc))
        self.twist = Twist(
            self.twist.linear + a * (raw.linear - self.twist.linear),
            self.twist.angular + a * (raw.angular - self.twist.angular),
        )

    def reset(self, pose: Pose2D | None = None) -> None:
        self.pose = pose or Pose2D()
        self.twist = Twist()
        self.twist_raw = Twist()
        self._prev = {"left": None, "right": None}
        self.total_distance = 0.0

    @classmethod
    def _delta_counts(cls, prev: int, now: int) -> int:
        d = (now - prev) % cls.INT32_SPAN
        if d >= cls.INT32_SPAN // 2:
            d -= cls.INT32_SPAN
        return d

    def _counts_to_meters(self, counts: float, side: str) -> float:
        circ = 2.0 * math.pi * self.hw.wheel_radius(side)
        return counts / self.hw.counts_per_wheel_rev * circ

    def update(self, left_counts: int, right_counts: int, dt: float) -> Pose2D:
        if self._prev["left"] is None:
            self._prev["left"] = left_counts
            self._prev["right"] = right_counts
            return self.pose

        dl_c = self._delta_counts(self._prev["left"], left_counts)
        dr_c = self._delta_counts(self._prev["right"], right_counts)
        self._prev["left"] = left_counts
        self._prev["right"] = right_counts

        dl = self._counts_to_meters(dl_c, "left")
        dr = self._counts_to_meters(dr_c, "right")

        ds = (dl + dr) / 2.0 * self.hw.linear_scale
        dth = (dr - dl) / self.hw.wheel_separation_m * self.hw.angular_scale

        # 원호(exact) 적분 — 직선 근사보다 회전 중 오차가 훨씬 작다
        th = self.pose.theta
        if abs(dth) < 1e-9:
            self.pose.x += ds * math.cos(th)
            self.pose.y += ds * math.sin(th)
        else:
            r = ds / dth
            self.pose.x += r * (math.sin(th + dth) - math.sin(th))
            self.pose.y += -r * (math.cos(th + dth) - math.cos(th))
        self.pose.theta = normalize_angle(th + dth)

        self.total_distance += abs(ds)
        if dt > 0:
            self._filter_twist(Twist(ds / dt, dth / dt), dt)
        return self.pose

    def update_from_velocity(self, left_mps: float, right_mps: float,
                             dt: float) -> Pose2D:
        """엔코더 위치가 없을 때(속도 PDO만 있을 때) 대체 경로."""
        tw = self.kin.wheel_mps_to_twist(left_mps, right_mps)
        ds = tw.linear * dt * self.hw.linear_scale
        dth = tw.angular * dt * self.hw.angular_scale
        th = self.pose.theta
        if abs(dth) < 1e-9:
            self.pose.x += ds * math.cos(th)
            self.pose.y += ds * math.sin(th)
        else:
            r = ds / dth
            self.pose.x += r * (math.sin(th + dth) - math.sin(th))
            self.pose.y += -r * (math.cos(th + dth) - math.cos(th))
        self.pose.theta = normalize_angle(th + dth)
        self.total_distance += abs(ds)
        self._filter_twist(tw, dt)
        return self.pose

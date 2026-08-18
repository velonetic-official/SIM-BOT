"""모터 + 엔코더 물리 모델 (1축)."""
from __future__ import annotations

import random


class MotorModel:
    """속도 지령에 1차 지연으로 추종하는 모터 + 적산 엔코더.

    실 드라이브의 속도루프를 간단히 흉내낸다.
    파라미터는 hardware.yaml 의 simulation 섹션에서 온다.
    """

    def __init__(self, counts_per_motor_rev: int, tau_s: float = 0.08,
                 max_rpm: float = 3000.0, noise_counts: float = 0.0,
                 slip: float = 0.0, accel_rpm_s: float = 20000.0):
        self.cpr = counts_per_motor_rev
        self.tau = max(1e-3, tau_s)
        self.max_rpm = max_rpm
        self.noise = noise_counts
        self.slip = slip
        self.accel_rpm_s = accel_rpm_s

        self.target_rpm = 0.0     # 상위가 준 목표 (0x60FF)
        self.ramp_rpm = 0.0       # 프로파일 발생기 출력 (0x6083/0x6084 적용 후)
        self.actual_rpm = 0.0     # 속도루프 응답
        self.position_counts = 0.0
        self.enabled = False

    def set_target_rpm(self, rpm: float) -> None:
        self.target_rpm = max(-self.max_rpm, min(self.max_rpm, rpm))

    def step(self, dt: float) -> None:
        goal = self.target_rpm if self.enabled else 0.0

        # 1단계: 가속/감속 프로파일 발생기 (0x6083 / 0x6084)
        max_delta = self.accel_rpm_s * dt
        err = goal - self.ramp_rpm
        self.ramp_rpm += max(-max_delta, min(max_delta, err))

        # 2단계: 속도루프 1차 지연 (램프를 추종)
        self.actual_rpm += (self.ramp_rpm - self.actual_rpm) * min(1.0, dt / self.tau)

        effective = self.actual_rpm * (1.0 - self.slip)
        self.position_counts += effective / 60.0 * self.cpr * dt

    def encoder_counts(self) -> int:
        n = self.position_counts
        if self.noise > 0:
            n += random.gauss(0.0, self.noise)
        # 드라이브가 i32 로 리포트한다고 가정 (랩어라운드 재현)
        return int(n) - ((int(n) + 2**31) // 2**32) * 2**32

    def velocity_rpm(self) -> float:
        return self.actual_rpm

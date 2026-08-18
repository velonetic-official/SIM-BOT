"""설정 로더 + 검증 + 파생 상수 계산.

hardware.yaml / bus.yaml / profiles/*.yaml 을 읽어 dataclass 로 만든다.
'실제 하드웨어 값만 바꾸면 동작' 을 보장하기 위해, 스케일 계산은 전부 여기 모아둔다.
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass, field
from typing import Any, Dict, List

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG_DIR = os.path.join(ROOT, "config")


# --------------------------------------------------------------------------- #
# YAML 헬퍼: 0x.. 문자열/정수 모두 허용
# --------------------------------------------------------------------------- #
def as_int(v: Any) -> int:
    if isinstance(v, bool):
        return int(v)
    if isinstance(v, int):
        return v
    if isinstance(v, str):
        return int(v, 0)
    raise TypeError(f"정수로 해석할 수 없음: {v!r}")


def load_yaml(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


# --------------------------------------------------------------------------- #
# 하드웨어 제원
# --------------------------------------------------------------------------- #
@dataclass
class WheelSpec:
    axis: int
    invert_command: bool
    invert_feedback: bool


@dataclass
class HardwareConfig:
    raw: dict

    # 기구
    wheel_radius_m: float = 0.0
    wheel_separation_m: float = 0.0
    linear_scale: float = 1.0
    angular_scale: float = 1.0
    wheel_radius_left_ratio: float = 1.0

    # 드라이브트레인
    gear_ratio: float = 1.0
    counts_per_motor_rev: int = 0
    counts_are_post_gear: bool = False
    left: WheelSpec = None       # type: ignore
    right: WheelSpec = None      # type: ignore

    # 한계
    max_motor_rpm: float = 0.0
    max_linear_mps: float = 0.0
    max_angular_rps: float = 0.0
    max_linear_accel: float = 0.0
    max_angular_accel: float = 0.0
    cmd_timeout_s: float = 0.5

    # CAN
    node_id: int = 1
    bitrate: int = 500000
    control_period_s: float = 0.02
    feedback_period_s: float = 0.02
    heartbeat_s: float = 0.2
    sync_enable: bool = False
    odom_lpf_hz: float = 5.0

    # 시뮬
    sim: dict = field(default_factory=dict)

    # ---- 파생값 -----------------------------------------------------------
    @property
    def counts_per_wheel_rev(self) -> float:
        """휠 1회전당 엔코더 카운트."""
        if self.counts_are_post_gear:
            return float(self.counts_per_motor_rev)
        return float(self.counts_per_motor_rev) * self.gear_ratio

    @property
    def meters_per_count(self) -> float:
        return (2.0 * math.pi * self.wheel_radius_m) / self.counts_per_wheel_rev

    @property
    def max_wheel_rpm(self) -> float:
        return self.max_motor_rpm / self.gear_ratio

    def wheel_radius(self, side: str) -> float:
        r = self.wheel_radius_m
        return r * self.wheel_radius_left_ratio if side == "left" else r

    def motor_rpm_from_wheel_mps(self, mps: float, side: str) -> float:
        """휠 선속도(m/s) → 모터 rpm."""
        circ = 2.0 * math.pi * self.wheel_radius(side)
        wheel_rps = mps / circ
        return wheel_rps * 60.0 * self.gear_ratio

    def wheel_mps_from_motor_rpm(self, rpm: float, side: str) -> float:
        circ = 2.0 * math.pi * self.wheel_radius(side)
        return (rpm / 60.0 / self.gear_ratio) * circ

    def spec(self, side: str) -> WheelSpec:
        return self.left if side == "left" else self.right

    # ---- 로더 -------------------------------------------------------------
    @classmethod
    def load(cls, path: str | None = None) -> "HardwareConfig":
        path = path or os.path.join(CONFIG_DIR, "hardware.yaml")
        d = load_yaml(path)
        rb, dt, li, cn = d["robot"], d["drivetrain"], d["limits"], d["can"]
        cal = rb.get("calibration", {})
        cfg = cls(
            raw=d,
            wheel_radius_m=float(rb["wheel_radius_m"]),
            wheel_separation_m=float(rb["wheel_separation_m"]),
            linear_scale=float(cal.get("linear_scale", 1.0)),
            angular_scale=float(cal.get("angular_scale", 1.0)),
            wheel_radius_left_ratio=float(cal.get("wheel_radius_left_ratio", 1.0)),
            gear_ratio=float(dt["gear_ratio"]),
            counts_per_motor_rev=as_int(dt["encoder_counts_per_motor_rev"]),
            counts_are_post_gear=bool(dt.get("encoder_counts_are_post_gear", False)),
            left=WheelSpec(as_int(dt["left"]["axis"]),
                           bool(dt["left"]["invert_command"]),
                           bool(dt["left"]["invert_feedback"])),
            right=WheelSpec(as_int(dt["right"]["axis"]),
                            bool(dt["right"]["invert_command"]),
                            bool(dt["right"]["invert_feedback"])),
            max_motor_rpm=float(li["max_motor_rpm"]),
            max_linear_mps=float(li["max_linear_mps"]),
            max_angular_rps=float(li["max_angular_rps"]),
            max_linear_accel=float(li["max_linear_accel"]),
            max_angular_accel=float(li["max_angular_accel"]),
            cmd_timeout_s=float(li["cmd_timeout_s"]),
            node_id=as_int(cn["node_id"]),
            bitrate=as_int(cn["bitrate"]),
            control_period_s=as_int(cn["control_period_ms"]) / 1000.0,
            feedback_period_s=as_int(cn["feedback_period_ms"]) / 1000.0,
            heartbeat_s=as_int(cn["heartbeat_ms"]) / 1000.0,
            sync_enable=bool(cn.get("sync_enable", False)),
            odom_lpf_hz=float(cn.get("odom_velocity_lpf_hz", 5.0)),
            sim=d.get("simulation", {}),
        )
        cfg.validate()
        return cfg

    def validate(self) -> None:
        errs: List[str] = []
        if self.wheel_radius_m <= 0:
            errs.append("robot.wheel_radius_m 은 0보다 커야 합니다")
        if self.wheel_separation_m <= 0:
            errs.append("robot.wheel_separation_m 은 0보다 커야 합니다")
        if self.gear_ratio <= 0:
            errs.append("drivetrain.gear_ratio 은 0보다 커야 합니다")
        if self.counts_per_motor_rev <= 0:
            errs.append("drivetrain.encoder_counts_per_motor_rev 은 0보다 커야 합니다")
        if self.left.axis == self.right.axis:
            errs.append("좌/우 axis 번호가 같습니다")
        if self.feedback_period_s > self.control_period_s / 2.0 + 1e-9:
            errs.append(
                f"can.feedback_period_ms({self.feedback_period_s*1000:.0f}) 는 "
                f"can.control_period_ms({self.control_period_s*1000:.0f}) 의 "
                f"절반 이하여야 합니다 (속도추정 리플 방지)")
        if not (1 <= self.node_id <= 127):
            errs.append("can.node_id 는 1~127 이어야 합니다")
        # 기구학적으로 도달 불가능한 최대속도 지정 방지
        reachable = self.wheel_mps_from_motor_rpm(self.max_motor_rpm, "right")
        if self.max_linear_mps > reachable + 1e-9:
            errs.append(
                f"limits.max_linear_mps({self.max_linear_mps:.3f}) 가 모터 최대회전수로 "
                f"낼 수 있는 속도({reachable:.3f} m/s)를 초과합니다")
        if errs:
            raise ValueError("hardware.yaml 오류:\n  - " + "\n  - ".join(errs))

    def summary(self) -> str:
        return (
            f"[하드웨어 제원 요약]  status={self.raw['meta'].get('status')}\n"
            f"  휠 반지름          : {self.wheel_radius_m*1000:.1f} mm\n"
            f"  트레드             : {self.wheel_separation_m*1000:.1f} mm\n"
            f"  감속비             : {self.gear_ratio:.2f} : 1\n"
            f"  엔코더             : {self.counts_per_motor_rev} counts/모터rev "
            f"→ {self.counts_per_wheel_rev:.0f} counts/휠rev\n"
            f"  분해능             : {self.meters_per_count*1000:.5f} mm / count\n"
            f"  최대 휠 rpm        : {self.max_wheel_rpm:.1f} "
            f"(= {self.wheel_mps_from_motor_rpm(self.max_motor_rpm,'right'):.3f} m/s)\n"
            f"  CAN                : node {self.node_id}, {self.bitrate/1000:.0f} kbit/s, "
            f"제어주기 {self.control_period_s*1000:.0f} ms"
        )


# --------------------------------------------------------------------------- #
# 버스 설정
# --------------------------------------------------------------------------- #
@dataclass
class BusConfig:
    backend: str
    kwargs: Dict[str, Any]
    trace_enabled: bool = True
    trace_max: int = 4000

    @classmethod
    def load(cls, path: str | None = None, override_backend: str | None = None,
             bitrate: int | None = None) -> "BusConfig":
        path = path or os.path.join(CONFIG_DIR, "bus.yaml")
        d = load_yaml(path)
        backend = override_backend or d["backend"]
        if backend not in d["profiles"]:
            raise ValueError(f"bus.yaml 에 '{backend}' 프로파일이 없습니다. "
                             f"사용 가능: {list(d['profiles'])}")
        kw = dict(d["profiles"][backend])
        if bitrate and "bitrate" in kw:
            kw["bitrate"] = bitrate
        tr = d.get("trace", {})
        return cls(backend=backend, kwargs=kw,
                   trace_enabled=bool(tr.get("enabled", True)),
                   trace_max=as_int(tr.get("max_frames", 4000)))


# --------------------------------------------------------------------------- #
# 프로파일(오브젝트 딕셔너리 + PDO 매핑)
# --------------------------------------------------------------------------- #
@dataclass
class ObjEntry:
    index: int
    sub: int
    type: str


@dataclass
class PdoField:
    signal: str
    axis: int
    type: str


@dataclass
class PdoMap:
    name: str
    cob_base: int
    fields: List[PdoField]


@dataclass
class ProfileConfig:
    raw: dict
    name: str
    axis_index_offset: int
    velocity_unit: str
    velocity_scale: float
    objects: Dict[str, ObjEntry]
    modes: Dict[str, int]
    controlword: Dict[str, int]
    statusword_states: Dict[str, tuple]
    rx_pdos: List[PdoMap]
    tx_pdos: List[PdoMap]
    sdo_startup: List[dict]
    cob: Dict[str, int]
    nmt_commands: Dict[str, int]

    @classmethod
    def load(cls, path: str | None = None) -> "ProfileConfig":
        path = path or os.path.join(CONFIG_DIR, "profiles",
                                    "cia402_leadshine_2eld2.yaml")
        d = load_yaml(path)
        p, u = d["profile"], d["units"]

        def mk_pdos(items):
            return [PdoMap(i["name"], as_int(i["cob_base"]),
                           [PdoField(f["signal"], as_int(f["axis"]), f["type"])
                            for f in i["fields"]]) for i in items]

        return cls(
            raw=d,
            name=p["name"],
            axis_index_offset=as_int(p["axis_index_offset"]),
            velocity_unit=u["velocity"],
            velocity_scale=float(u.get("velocity_scale", 1.0)),
            objects={k: ObjEntry(as_int(v["index"]), as_int(v["sub"]), v["type"])
                     for k, v in d["objects"].items()},
            modes={k: as_int(v) for k, v in d["modes"].items()},
            controlword={k: as_int(v) for k, v in d["controlword"].items()},
            statusword_states={k: (as_int(v[0]), as_int(v[1]))
                               for k, v in d["statusword_states"].items()},
            rx_pdos=mk_pdos(d["pdo"]["rx"]),
            tx_pdos=mk_pdos(d["pdo"]["tx"]),
            sdo_startup=d.get("sdo_startup", []),
            cob={k: as_int(v) for k, v in d["cob"].items()},
            nmt_commands={k: as_int(v) for k, v in d["nmt_commands"].items()},
        )

    # 축 번호를 반영한 실제 인덱스
    def entry(self, signal: str, axis: int) -> ObjEntry:
        e = self.objects[signal]
        off = self.axis_index_offset * (axis - 1) if axis >= 1 else 0
        return ObjEntry(e.index + off, e.sub, e.type)

    def state_of(self, statusword: int) -> str:
        for name, (mask, val) in self.statusword_states.items():
            if statusword & mask == val:
                return name
        return "unknown"


@dataclass
class Settings:
    hw: HardwareConfig
    bus: BusConfig
    profile: ProfileConfig

    @classmethod
    def load(cls, backend: str | None = None,
             hardware_path: str | None = None,
             profile_path: str | None = None) -> "Settings":
        hw = HardwareConfig.load(hardware_path)
        bus = BusConfig.load(override_backend=backend, bitrate=hw.bitrate)
        prof = ProfileConfig.load(profile_path)
        return cls(hw=hw, bus=bus, profile=prof)

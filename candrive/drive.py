"""2ELD2-CAN (2축 통합 드라이브) 마스터측 클라이언트.

- NMT 기동 → SDO 초기화 → CiA402 상태머신 enable → PDO 주기 제어
- 축1/축2 를 좌/우 휠에 매핑 (hardware.yaml 의 drivetrain.left/right.axis)
- 시뮬이든 실물이든 동일 코드 경로. 다른 것은 CanLink 백엔드뿐.
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

import can

from . import canopen as co
from .bus import CanLink
from .config import HardwareConfig, ProfileConfig


@dataclass
class AxisState:
    statusword: int = 0
    state: str = "unknown"
    actual_velocity_raw: int = 0
    actual_position_raw: int = 0
    error_code: int = 0
    last_update: float = 0.0


@dataclass
class DriveTelemetry:
    axes: Dict[int, AxisState] = field(default_factory=dict)
    heartbeat_state: int = -1
    last_heartbeat: float = 0.0
    emcy: Optional[Tuple[int, bytes]] = None

    def online(self, timeout: float = 1.0) -> bool:
        return (time.monotonic() - self.last_heartbeat) < timeout


class SdoTimeout(RuntimeError):
    pass


class DualAxisDrive:
    """마스터측 드라이브 핸들."""

    def __init__(self, link: CanLink, hw: HardwareConfig, profile: ProfileConfig):
        self.link = link
        self.hw = hw
        self.p = profile
        self.node_id = hw.node_id
        self.rx_codecs, self.tx_codecs = co.build_codecs(profile, self.node_id)
        self.telemetry = DriveTelemetry(axes={1: AxisState(), 2: AxisState()})
        self._sdo_resp: Dict[Tuple[int, int], Tuple[int, Optional[int]]] = {}
        self._sdo_event = threading.Event()
        self._targets: Dict[Tuple[str, int], int] = {}
        self._lock = threading.Lock()

        link.labeler = co.make_labeler(profile, self.node_id)
        link.add_handler(self._on_message)
        link.start_rx_thread()

    # ------------------------------------------------------------------ 수신
    def _on_message(self, msg: can.Message) -> None:
        cid = msg.arbitration_id
        data = bytes(msg.data)
        c = self.p.cob

        if cid == c["heartbeat_base"] + self.node_id and len(data) >= 1:
            self.telemetry.heartbeat_state = data[0]
            self.telemetry.last_heartbeat = time.monotonic()
            return

        if cid == c["sdo_tx_base"] + self.node_id and len(data) >= 4:
            cmd, index, sub, raw = co.parse_sdo(data)
            self._sdo_resp[(index, sub)] = (cmd, raw)
            self._sdo_event.set()
            return

        if cid == c["emergency_base"] + self.node_id and len(data) >= 2:
            self.telemetry.emcy = (data[0] | (data[1] << 8), data)
            return

        for codec in self.tx_codecs:
            if cid == codec.cob_id:
                vals = codec.unpack(data)
                now = time.monotonic()
                for (sig, axis), v in vals.items():
                    st = self.telemetry.axes.setdefault(axis, AxisState())
                    if sig == "statusword":
                        st.statusword = v
                        st.state = self.p.state_of(v)
                    elif sig == "actual_velocity":
                        st.actual_velocity_raw = v
                    elif sig == "actual_position":
                        st.actual_position_raw = v
                    elif sig == "error_code":
                        st.error_code = v
                    st.last_update = now
                return

    # ------------------------------------------------------------------ SDO
    def sdo_write(self, signal: str, axis: int, value: int,
                  timeout: float = 0.5) -> None:
        e = self.p.entry(signal, axis)
        self._sdo_resp.pop((e.index, e.sub), None)
        self._sdo_event.clear()
        frame = co.sdo_download(e.index, e.sub, e.type, value)
        self.link.send(self.p.cob["sdo_rx_base"] + self.node_id, frame)
        self._await_sdo(e.index, e.sub, timeout, f"쓰기 {signal}(축{axis})")

    def sdo_read(self, signal: str, axis: int, timeout: float = 0.5) -> int:
        e = self.p.entry(signal, axis)
        self._sdo_resp.pop((e.index, e.sub), None)
        self._sdo_event.clear()
        self.link.send(self.p.cob["sdo_rx_base"] + self.node_id,
                       co.sdo_upload_request(e.index, e.sub))
        cmd, raw = self._await_sdo(e.index, e.sub, timeout,
                                   f"읽기 {signal}(축{axis})")
        return co.decode(e.type, (raw or 0).to_bytes(4, "little"))

    def _await_sdo(self, index: int, sub: int, timeout: float, what: str):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            got = self._sdo_resp.get((index, sub))
            if got is not None:
                cmd, raw = got
                if cmd == co.SDO_ABORT:
                    raise RuntimeError(
                        f"SDO abort ({what}) idx=0x{index:04X}:{sub:02X} "
                        f"code=0x{(raw or 0):08X}")
                return got
            self._sdo_event.wait(0.005)
            self._sdo_event.clear()
        raise SdoTimeout(
            f"SDO 응답 없음 ({what}) idx=0x{index:04X}:{sub:02X}. "
            f"노드 ID/보레이트/종단저항을 확인하세요.")

    # ------------------------------------------------------------------ NMT
    def nmt(self, command: str) -> None:
        self.link.send(self.p.cob["nmt"],
                       co.nmt_frame(self.p.nmt_commands[command], self.node_id))

    # ------------------------------------------------------------- 기동 절차
    def bringup(self, verbose: bool = True) -> None:
        log = print if verbose else (lambda *a, **k: None)
        log(f"[drive] NMT reset_comm → pre_operational (node {self.node_id})")
        self.nmt("reset_comm")
        time.sleep(0.05)
        self.nmt("pre_operational")
        time.sleep(0.05)

        for step in self.p.sdo_startup:
            axes = [1, 2] if int(step.get("axis", 0)) == 0 else [int(step["axis"])]
            for ax in axes:
                self.sdo_write(step["signal"], ax, int(step["value"]))
            log(f"[drive] SDO  {step['desc']:<24} = {step['value']}")

        log("[drive] NMT start (operational)")
        self.nmt("start")
        time.sleep(0.05)

    def enable(self, timeout: float = 2.0, verbose: bool = True) -> None:
        """CiA402 상태천이: switch_on_disabled → operation_enabled."""
        log = print if verbose else (lambda *a, **k: None)
        cw = self.p.controlword
        seq = [("fault_reset", cw["fault_reset"]),
               ("shutdown", cw["shutdown"]),
               ("switch_on", cw["switch_on"]),
               ("enable_operation", cw["enable_operation"])]
        for name, word in seq:
            with self._lock:
                self._targets[("controlword", 1)] = word
                self._targets[("controlword", 2)] = word
            self._send_rpdos()
            time.sleep(0.05)
            log(f"[drive] CW {name:<18} 0x{word:04X} → "
                f"축1={self.telemetry.axes[1].state} 축2={self.telemetry.axes[2].state}")

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if all(self.telemetry.axes[a].state == "operation_enabled"
                   for a in (1, 2)):
                log("[drive] ✔ 양축 operation_enabled")
                return
            self._send_rpdos()
            time.sleep(0.02)
        raise RuntimeError(
            "드라이브 enable 실패: "
            + ", ".join(f"축{a}={self.telemetry.axes[a].state}" for a in (1, 2)))

    def disable(self) -> None:
        self.set_wheel_rpm(0.0, 0.0)
        with self._lock:
            self._targets[("controlword", 1)] = self.p.controlword["shutdown"]
            self._targets[("controlword", 2)] = self.p.controlword["shutdown"]
        self._send_rpdos()

    # ------------------------------------------------------- 속도 지령/피드백
    def _velocity_raw(self, motor_rpm: float) -> int:
        """모터 rpm → 드라이브 원시 정수 (프로파일 units 에 따름)."""
        u = self.p.velocity_unit
        if u == "rpm":
            v = motor_rpm
        elif u == "rpm_x10":
            v = motor_rpm * 10.0
        elif u == "counts_per_sec":
            v = motor_rpm / 60.0 * self.hw.counts_per_motor_rev
        else:
            raise ValueError(f"알 수 없는 velocity 단위: {u}")
        return int(round(v * self.p.velocity_scale))

    def _rpm_from_raw(self, raw: int) -> float:
        u = self.p.velocity_unit
        v = raw / self.p.velocity_scale
        if u == "rpm":
            return v
        if u == "rpm_x10":
            return v / 10.0
        if u == "counts_per_sec":
            return v * 60.0 / self.hw.counts_per_motor_rev
        raise ValueError(f"알 수 없는 velocity 단위: {u}")

    def set_wheel_rpm(self, left_wheel_rpm: float, right_wheel_rpm: float) -> None:
        """휠 rpm 지령 (감속비/부호반전은 여기서 흡수)."""
        with self._lock:
            for side, wheel_rpm in (("left", left_wheel_rpm),
                                    ("right", right_wheel_rpm)):
                spec = self.hw.spec(side)
                motor_rpm = wheel_rpm * self.hw.gear_ratio
                motor_rpm = max(-self.hw.max_motor_rpm,
                                min(self.hw.max_motor_rpm, motor_rpm))
                if spec.invert_command:
                    motor_rpm = -motor_rpm
                self._targets[("target_velocity", spec.axis)] = \
                    self._velocity_raw(motor_rpm)
                self._targets[("controlword", spec.axis)] = \
                    self.p.controlword["enable_operation"]

    def wheel_feedback(self) -> Dict[str, Dict[str, float]]:
        """피드백을 '휠 단위'로 환산해서 반환."""
        out: Dict[str, Dict[str, float]] = {}
        for side in ("left", "right"):
            spec = self.hw.spec(side)
            st = self.telemetry.axes.get(spec.axis, AxisState())
            sign = -1.0 if spec.invert_feedback else 1.0
            motor_rpm = self._rpm_from_raw(st.actual_velocity_raw) * sign
            counts = st.actual_position_raw * sign
            out[side] = {
                "wheel_rpm": motor_rpm / self.hw.gear_ratio,
                "mps": self.hw.wheel_mps_from_motor_rpm(motor_rpm, side),
                "counts": counts,
                "state": st.state,
                "statusword": st.statusword,
            }
        return out

    # ----------------------------------------------------------- 주기 송신
    def _send_rpdos(self) -> None:
        with self._lock:
            snapshot = dict(self._targets)
        for codec in self.rx_codecs:
            self.link.send(codec.cob_id, codec.pack(snapshot))

    def tick(self) -> None:
        """제어 주기마다 1회 호출."""
        self._send_rpdos()

    def close(self) -> None:
        try:
            self.disable()
            self.nmt("pre_operational")
        except Exception:      # noqa: BLE001
            pass

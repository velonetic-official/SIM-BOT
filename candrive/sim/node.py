"""가상 2ELD2-CAN 드라이브 (CANopen 슬레이브 시뮬레이터).

실제 드라이브 대신 버스에 붙어서:
  - NMT 수신 → 상태 전이, 하트비트 송신
  - SDO 요청 → 오브젝트 딕셔너리 읽기/쓰기 응답
  - RPDO 수신 → 컨트롤워드/목표속도 반영
  - TPDO 주기 송신 → statusword / actual velocity / actual position
  - CiA402 상태머신 재현 (shutdown→switched_on→operation_enabled)

★ 이 파일은 "하드웨어 대역" 이므로 실물이 오면 그냥 실행하지 않으면 된다.
   마스터 코드(drive.py)는 전혀 바뀌지 않는다.
"""
from __future__ import annotations

import threading
import time
from typing import Dict, Tuple

import can

from .. import canopen as co
from ..bus import CanLink
from ..config import HardwareConfig, ProfileConfig
from .motor import MotorModel


class VirtualDrive:
    def __init__(self, link: CanLink, hw: HardwareConfig, profile: ProfileConfig):
        self.link = link
        self.hw = hw
        self.p = profile
        self.node_id = hw.node_id
        self.rx_codecs, self.tx_codecs = co.build_codecs(profile, self.node_id)

        sim = hw.sim
        self.motors: Dict[int, MotorModel] = {
            ax: MotorModel(
                counts_per_motor_rev=hw.counts_per_motor_rev,
                tau_s=float(sim.get("motor_time_constant_s", 0.08)),
                max_rpm=hw.max_motor_rpm,
                noise_counts=float(sim.get("encoder_noise_counts", 0.0)),
                slip=float(sim.get("wheel_slip_ratio", 0.0)) if ax == 1 else 0.0,
            ) for ax in (1, 2)
        }

        # 오브젝트 딕셔너리 저장소: (index, sub) -> value
        self.od: Dict[Tuple[int, int], int] = {}
        self._init_od()

        self.nmt_state = 0x7F           # pre-operational
        self.cia402: Dict[int, str] = {1: "switch_on_disabled",
                                       2: "switch_on_disabled"}
        self.latency = float(sim.get("bus_latency_ms", 0.0)) / 1000.0

        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._last_step = time.monotonic()

        link.labeler = co.make_labeler(profile, self.node_id)
        link.add_handler(self._on_message)
        link.start_rx_thread()

    # ------------------------------------------------------------------ OD
    def _init_od(self) -> None:
        for ax in (1, 2):
            for sig in ("controlword", "statusword", "modes_of_operation",
                        "modes_display", "target_velocity", "actual_velocity",
                        "actual_position", "profile_accel", "profile_decel",
                        "quick_stop_decel", "error_code"):
                if sig not in self.p.objects:
                    continue
                e = self.p.entry(sig, ax)
                self.od[(e.index, e.sub)] = 0
            self.od[(self.p.entry("statusword", ax).index, 0)] = 0x0040

    def _lookup(self, index: int, sub: int) -> Tuple[str, int] | None:
        """(signal, axis) 역조회."""
        for ax in (1, 2):
            for sig in self.p.objects:
                e = self.p.entry(sig, ax)
                if e.index == index and e.sub == sub:
                    return sig, ax
        return None

    # ------------------------------------------------------------- 상태머신
    def _statusword(self, axis: int) -> int:
        table = {
            "switch_on_disabled": 0x0040,
            "ready_to_switch_on": 0x0021,
            "switched_on":        0x0023,
            "operation_enabled":  0x0237,   # + target reached / speed bits
            "fault":              0x0008,
        }
        return table.get(self.cia402[axis], 0x0000)

    def _apply_controlword(self, axis: int, cw: int) -> None:
        st = self.cia402[axis]
        low = cw & 0x008F
        if cw & self.p.controlword["fault_reset"] and st == "fault":
            self.cia402[axis] = "switch_on_disabled"
            return
        if low == self.p.controlword["shutdown"]:
            if st in ("switch_on_disabled", "switched_on", "operation_enabled",
                      "ready_to_switch_on"):
                self.cia402[axis] = "ready_to_switch_on"
        elif low == self.p.controlword["switch_on"]:
            if st in ("ready_to_switch_on", "switched_on"):
                self.cia402[axis] = "switched_on"
        elif low == self.p.controlword["enable_operation"]:
            if st in ("switched_on", "operation_enabled"):
                self.cia402[axis] = "operation_enabled"
        elif low == self.p.controlword["disable_voltage"]:
            self.cia402[axis] = "switch_on_disabled"
        self.motors[axis].enabled = (self.cia402[axis] == "operation_enabled")

    # ------------------------------------------------------------------ RX
    def _on_message(self, msg: can.Message) -> None:
        if self.latency:
            time.sleep(self.latency)
        cid = msg.arbitration_id
        data = bytes(msg.data)
        c = self.p.cob

        # NMT
        if cid == c["nmt"] and len(data) >= 2:
            cmd, nid = data[0], data[1]
            if nid in (0, self.node_id):
                if cmd == self.p.nmt_commands["start"]:
                    self.nmt_state = 0x05
                elif cmd == self.p.nmt_commands["pre_operational"]:
                    self.nmt_state = 0x7F
                elif cmd == self.p.nmt_commands["stop"]:
                    self.nmt_state = 0x04
                elif cmd in (self.p.nmt_commands["reset_node"],
                             self.p.nmt_commands["reset_comm"]):
                    self.nmt_state = 0x00
                    with self._lock:
                        for ax in (1, 2):
                            self.cia402[ax] = "switch_on_disabled"
                            self.motors[ax].enabled = False
                            self.motors[ax].set_target_rpm(0.0)
                    self.nmt_state = 0x7F
            return

        # SDO 요청
        if cid == c["sdo_rx_base"] + self.node_id and len(data) >= 4:
            self._handle_sdo(data)
            return

        # RPDO (operational 에서만 수신)
        if self.nmt_state != 0x05:
            return
        for codec in self.rx_codecs:
            if cid == codec.cob_id:
                vals = codec.unpack(data)
                with self._lock:
                    for (sig, axis), v in vals.items():
                        if sig == "controlword":
                            self._apply_controlword(axis, v)
                        elif sig == "target_velocity":
                            self.od[(self.p.entry(sig, axis).index, 0)] = v
                            self.motors[axis].set_target_rpm(
                                self._raw_to_rpm(v))
                return

    def _raw_to_rpm(self, raw: int) -> float:
        u, s = self.p.velocity_unit, self.p.velocity_scale
        v = raw / s
        if u == "rpm":
            return v
        if u == "rpm_x10":
            return v / 10.0
        if u == "counts_per_sec":
            return v * 60.0 / self.hw.counts_per_motor_rev
        raise ValueError(u)

    def _rpm_to_raw(self, rpm: float) -> int:
        u, s = self.p.velocity_unit, self.p.velocity_scale
        if u == "rpm":
            v = rpm
        elif u == "rpm_x10":
            v = rpm * 10.0
        elif u == "counts_per_sec":
            v = rpm / 60.0 * self.hw.counts_per_motor_rev
        else:
            raise ValueError(u)
        return int(round(v * s))

    def _handle_sdo(self, data: bytes) -> None:
        cmd, index, sub, raw = co.parse_sdo(data)
        tx = self.p.cob["sdo_tx_base"] + self.node_id
        hit = self._lookup(index, sub)
        if hit is None:
            self.link.send(tx, co.sdo_abort(index, sub, 0x06020000))
            return
        sig, axis = hit
        dtype = self.p.objects[sig].type

        if cmd == 0x40:                              # upload (read)
            value = self.od.get((index, sub), 0)
            self.link.send(tx, co.sdo_upload_response(index, sub, dtype, value))
        elif cmd & 0xE0 == 0x20:                     # download (write)
            n = co.sdo_payload_size(cmd)
            value = co.decode(dtype, data[4:4 + max(n, co.type_size(dtype))])
            with self._lock:
                self.od[(index, sub)] = value
                if sig == "modes_of_operation":
                    e = self.p.entry("modes_display", axis)
                    self.od[(e.index, e.sub)] = value
                elif sig == "profile_accel":
                    self.motors[axis].accel_rpm_s = max(1.0, value / 10.0)
                elif sig == "controlword":
                    self._apply_controlword(axis, value)
                elif sig == "target_velocity":
                    self.motors[axis].set_target_rpm(self._raw_to_rpm(value))
            self.link.send(tx, co.sdo_download_response(index, sub))
        else:
            self.link.send(tx, co.sdo_abort(index, sub, 0x05040001))

    # ------------------------------------------------------------ 주기 태스크
    def _publish(self) -> None:
        with self._lock:
            values: Dict[Tuple[str, int], int] = {}
            for ax in (1, 2):
                m = self.motors[ax]
                sw = self._statusword(ax)
                self.od[(self.p.entry("statusword", ax).index, 0)] = sw
                self.od[(self.p.entry("actual_velocity", ax).index, 0)] = \
                    self._rpm_to_raw(m.velocity_rpm())
                self.od[(self.p.entry("actual_position", ax).index, 0)] = \
                    m.encoder_counts()
                values[("statusword", ax)] = sw
                values[("actual_velocity", ax)] = self._rpm_to_raw(m.velocity_rpm())
                values[("actual_position", ax)] = m.encoder_counts()
        for codec in self.tx_codecs:
            self.link.send(codec.cob_id, codec.pack(values))

    def _loop(self) -> None:
        period = self.hw.feedback_period_s
        hb_period = self.hw.heartbeat_s
        next_pub = time.monotonic()
        next_hb = time.monotonic()
        while not self._stop.is_set():
            now = time.monotonic()
            dt = now - self._last_step
            self._last_step = now
            with self._lock:
                for m in self.motors.values():
                    m.step(dt)
            if now >= next_hb:
                self.link.send(self.p.cob["heartbeat_base"] + self.node_id,
                               bytes([self.nmt_state]))
                next_hb = now + hb_period
            if now >= next_pub and self.nmt_state == 0x05:
                self._publish()
                next_pub = now + period
            time.sleep(0.002)

    def start(self) -> None:
        if self._thread:
            return
        self._last_step = time.monotonic()
        self._thread = threading.Thread(target=self._loop, daemon=True,
                                        name="virtual-drive")
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)

    # 시뮬 상태 조회 (지상검증용 — 마스터는 CAN 으로만 알 수 있음)
    def truth(self) -> dict:
        return {
            "nmt_state": self.nmt_state,
            "axes": {ax: {"state": self.cia402[ax],
                          "rpm": self.motors[ax].velocity_rpm(),
                          "counts": self.motors[ax].position_counts}
                     for ax in (1, 2)},
        }

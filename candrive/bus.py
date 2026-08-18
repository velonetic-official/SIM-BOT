"""CAN 백엔드 추상화 계층.

virtual / socketcan(vcan) / pcan 을 동일 인터페이스로 감싼다.
상위 코드는 backend 를 전혀 모르고 send()/recv() 만 쓴다.
=> bus.yaml 의 backend 한 줄만 바꾸면 시뮬 ↔ PCAN-USB 실물 전환.
"""
from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Callable, Deque, List, Optional

import can

from .config import BusConfig


@dataclass
class TracedFrame:
    t: float
    direction: str      # "TX" | "RX"
    cob_id: int
    data: bytes
    label: str = ""

    def to_dict(self) -> dict:
        return {
            "t": round(self.t, 4),
            "dir": self.direction,
            "id": f"0x{self.cob_id:03X}",
            "dlc": len(self.data),
            "data": self.data.hex(" ").upper(),
            "label": self.label,
        }


class CanLink:
    """python-can Bus 래퍼 + 프레임 트레이스 + 콜백 디스패치."""

    def __init__(self, cfg: BusConfig, name: str = "master"):
        self.cfg = cfg
        self.name = name
        kwargs = dict(cfg.kwargs)
        self._iface = kwargs.pop("interface")
        self.bus: can.BusABC = can.Bus(interface=self._iface, **kwargs)
        self.trace: Deque[TracedFrame] = deque(maxlen=cfg.trace_max)
        self._t0 = time.monotonic()
        self._handlers: List[Callable[[can.Message], None]] = []
        self._rx_thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self.tx_count = 0
        self.rx_count = 0
        self.labeler: Callable[[int, bytes], str] = lambda cid, d: ""

    # -- 송신 ---------------------------------------------------------------
    def send(self, cob_id: int, data: bytes, extended: bool = False) -> None:
        msg = can.Message(arbitration_id=cob_id, data=data,
                          is_extended_id=extended)
        with self._lock:
            self.bus.send(msg)
            self.tx_count += 1
        if self.cfg.trace_enabled:
            self.trace.append(TracedFrame(time.monotonic() - self._t0, "TX",
                                          cob_id, bytes(data),
                                          self.labeler(cob_id, bytes(data))))

    # -- 수신 ---------------------------------------------------------------
    def add_handler(self, fn: Callable[[can.Message], None]) -> None:
        self._handlers.append(fn)

    def recv(self, timeout: float = 0.0) -> Optional[can.Message]:
        msg = self.bus.recv(timeout=timeout)
        if msg is not None:
            self._note_rx(msg)
        return msg

    def _note_rx(self, msg: can.Message) -> None:
        self.rx_count += 1
        if self.cfg.trace_enabled:
            self.trace.append(TracedFrame(time.monotonic() - self._t0, "RX",
                                          msg.arbitration_id, bytes(msg.data),
                                          self.labeler(msg.arbitration_id,
                                                       bytes(msg.data))))

    def start_rx_thread(self) -> None:
        if self._rx_thread:
            return

        def loop():
            while not self._stop.is_set():
                msg = self.bus.recv(timeout=0.05)
                if msg is None:
                    continue
                self._note_rx(msg)
                for h in list(self._handlers):
                    try:
                        h(msg)
                    except Exception as exc:      # noqa: BLE001
                        print(f"[{self.name}] handler error: {exc}")

        self._rx_thread = threading.Thread(target=loop, daemon=True,
                                           name=f"canrx-{self.name}")
        self._rx_thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._rx_thread:
            self._rx_thread.join(timeout=1.0)
        try:
            self.bus.shutdown()
        except Exception:      # noqa: BLE001
            pass

    def recent(self, n: int = 50) -> List[dict]:
        return [f.to_dict() for f in list(self.trace)[-n:]]

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def describe(self) -> str:
        ch = self.cfg.kwargs.get("channel", "?")
        return f"backend={self.cfg.backend} interface={self._iface} channel={ch}"

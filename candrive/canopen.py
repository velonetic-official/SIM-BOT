"""CANopen 코덱: 데이터타입 인코딩, SDO, PDO 조립/해석, NMT.

프로파일 YAML 이 주는 정보만으로 프레임을 만든다.
매뉴얼이 확정되면 YAML 만 수정 → 이 파일은 그대로.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

from .config import ProfileConfig, PdoMap

# --------------------------------------------------------------------------- #
# 데이터 타입
# --------------------------------------------------------------------------- #
_FMT = {
    "u8": ("<B", 1), "i8": ("<b", 1),
    "u16": ("<H", 2), "i16": ("<h", 2),
    "u32": ("<I", 4), "i32": ("<i", 4),
}


def type_size(t: str) -> int:
    return _FMT[t][1]


def encode(t: str, value: int) -> bytes:
    fmt, size = _FMT[t]
    lo, hi = (0, (1 << (size * 8)) - 1) if t[0] == "u" else \
             (-(1 << (size * 8 - 1)), (1 << (size * 8 - 1)) - 1)
    v = max(lo, min(hi, int(round(value))))     # saturate (실드라이브 보호)
    return struct.pack(fmt, v)


def decode(t: str, data: bytes) -> int:
    fmt, size = _FMT[t]
    return struct.unpack(fmt, data[:size])[0]


# --------------------------------------------------------------------------- #
# SDO (expedited only — 4바이트 이하 오브젝트면 충분)
# --------------------------------------------------------------------------- #
SDO_ABORT = 0x80

_DL_CMD = {1: 0x2F, 2: 0x2B, 3: 0x27, 4: 0x23}


def sdo_download(index: int, sub: int, dtype: str, value: int) -> bytes:
    """마스터 → 드라이브 쓰기 요청."""
    payload = encode(dtype, value)
    n = len(payload)
    frame = bytearray(8)
    frame[0] = _DL_CMD[n]
    frame[1] = index & 0xFF
    frame[2] = (index >> 8) & 0xFF
    frame[3] = sub & 0xFF
    frame[4:4 + n] = payload
    return bytes(frame)


def sdo_upload_request(index: int, sub: int) -> bytes:
    """마스터 → 드라이브 읽기 요청."""
    return bytes([0x40, index & 0xFF, (index >> 8) & 0xFF, sub & 0xFF, 0, 0, 0, 0])


def sdo_upload_response(index: int, sub: int, dtype: str, value: int) -> bytes:
    payload = encode(dtype, value)
    n = len(payload)
    cmd = {1: 0x4F, 2: 0x4B, 3: 0x47, 4: 0x43}[n]
    frame = bytearray(8)
    frame[0] = cmd
    frame[1] = index & 0xFF
    frame[2] = (index >> 8) & 0xFF
    frame[3] = sub & 0xFF
    frame[4:4 + n] = payload
    return bytes(frame)


def sdo_download_response(index: int, sub: int) -> bytes:
    return bytes([0x60, index & 0xFF, (index >> 8) & 0xFF, sub & 0xFF, 0, 0, 0, 0])


def sdo_abort(index: int, sub: int, code: int = 0x06020000) -> bytes:
    return bytes([SDO_ABORT, index & 0xFF, (index >> 8) & 0xFF, sub & 0xFF]) + \
        struct.pack("<I", code)


def parse_sdo(data: bytes) -> Tuple[int, int, int, Optional[int]]:
    """(cmd, index, sub, raw_u32|None) 로 분해."""
    cmd = data[0]
    index = data[1] | (data[2] << 8)
    sub = data[3]
    raw = struct.unpack("<I", data[4:8])[0] if len(data) >= 8 else None
    return cmd, index, sub, raw


def sdo_payload_size(cmd: int) -> int:
    """expedited 커맨드 바이트에서 유효 데이터 길이 추출."""
    if cmd & 0x02:           # expedited
        n = (cmd >> 2) & 0x03
        return 4 - n
    return 4


# --------------------------------------------------------------------------- #
# PDO 조립/해석
# --------------------------------------------------------------------------- #
@dataclass
class PdoCodec:
    pmap: PdoMap
    node_id: int

    @property
    def cob_id(self) -> int:
        return self.pmap.cob_base + self.node_id

    def pack(self, values: Dict[Tuple[str, int], int]) -> bytes:
        out = bytearray()
        for f in self.pmap.fields:
            v = values.get((f.signal, f.axis), 0)
            out += encode(f.type, v)
        if len(out) > 8:
            raise ValueError(f"PDO '{self.pmap.name}' 가 8바이트를 초과합니다 "
                             f"({len(out)}B). 프로파일 YAML 의 fields 를 확인하세요.")
        return bytes(out)

    def unpack(self, data: bytes) -> Dict[Tuple[str, int], int]:
        out: Dict[Tuple[str, int], int] = {}
        off = 0
        for f in self.pmap.fields:
            sz = type_size(f.type)
            if off + sz > len(data):
                break
            out[(f.signal, f.axis)] = decode(f.type, data[off:off + sz])
            off += sz
        return out


def build_codecs(profile: ProfileConfig, node_id: int
                 ) -> Tuple[List[PdoCodec], List[PdoCodec]]:
    rx = [PdoCodec(p, node_id) for p in profile.rx_pdos]
    tx = [PdoCodec(p, node_id) for p in profile.tx_pdos]
    return rx, tx


# --------------------------------------------------------------------------- #
# 프레임 라벨러 (트레이스/대시보드 가독성용)
# --------------------------------------------------------------------------- #
def make_labeler(profile: ProfileConfig, node_id: int):
    known: Dict[int, str] = {profile.cob["nmt"]: "NMT",
                             profile.cob["sync"]: "SYNC"}
    known[profile.cob["sdo_rx_base"] + node_id] = "SDO req"
    known[profile.cob["sdo_tx_base"] + node_id] = "SDO resp"
    known[profile.cob["heartbeat_base"] + node_id] = "HEARTBEAT"
    known[profile.cob["emergency_base"] + node_id] = "EMCY"
    for p in profile.rx_pdos:
        known[p.cob_base + node_id] = f"RPDO {p.name}"
    for p in profile.tx_pdos:
        known[p.cob_base + node_id] = f"TPDO {p.name}"

    def labeler(cob_id: int, data: bytes) -> str:
        return known.get(cob_id, f"0x{cob_id:03X}")

    return labeler


# --------------------------------------------------------------------------- #
# NMT
# --------------------------------------------------------------------------- #
def nmt_frame(command: int, node_id: int = 0) -> bytes:
    return bytes([command & 0xFF, node_id & 0xFF])

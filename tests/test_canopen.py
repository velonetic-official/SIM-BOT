"""CANopen 코덱 / 프로파일 매핑 검증."""
import pytest

from candrive import canopen as co
from candrive.config import ProfileConfig, HardwareConfig


@pytest.fixture
def profile():
    return ProfileConfig.load()


@pytest.mark.parametrize("dtype,value", [
    ("u8", 0), ("u8", 255), ("i8", -128), ("i8", 127),
    ("u16", 0xFFFF), ("i16", -32768), ("u32", 0xDEADBEEF), ("i32", -123456789),
])
def test_encode_decode_roundtrip(dtype, value):
    assert co.decode(dtype, co.encode(dtype, value)) == value


def test_encode_saturates_instead_of_overflowing():
    """실드라이브에 말도 안 되는 값이 나가지 않도록 포화시킨다."""
    assert co.decode("i16", co.encode("i16", 999999)) == 32767
    assert co.decode("i16", co.encode("i16", -999999)) == -32768
    assert co.decode("u8", co.encode("u8", -5)) == 0


def test_axis_index_offset(profile):
    a1 = profile.entry("controlword", 1)
    a2 = profile.entry("controlword", 2)
    assert a2.index - a1.index == profile.axis_index_offset
    assert a1.index == 0x6040


def test_sdo_download_frame_shape(profile):
    e = profile.entry("target_velocity", 1)   # i32
    f = co.sdo_download(e.index, e.sub, e.type, -1500)
    assert len(f) == 8
    assert f[0] == 0x23                       # expedited, 4 bytes
    assert f[1] | (f[2] << 8) == 0x60FF
    assert co.decode("i32", f[4:8]) == -1500


def test_sdo_upload_request_and_response(profile):
    e = profile.entry("statusword", 2)
    req = co.sdo_upload_request(e.index, e.sub)
    assert req[0] == 0x40
    resp = co.sdo_upload_response(e.index, e.sub, e.type, 0x0237)
    cmd, idx, sub, raw = co.parse_sdo(resp)
    assert cmd == 0x4B                        # expedited, 2 bytes
    assert idx == 0x6841
    assert co.decode("u16", raw.to_bytes(4, "little")) == 0x0237


def test_pdo_pack_unpack_roundtrip(profile):
    rx, tx = co.build_codecs(profile, node_id=1)
    vel = next(c for c in rx if "velocity" in c.pmap.name)
    assert vel.cob_id == 0x301
    vals = {("target_velocity", 1): -1234, ("target_velocity", 2): 5678}
    data = vel.pack(vals)
    assert len(data) == 8
    assert vel.unpack(data) == vals


def test_pdo_rejects_over_8_bytes(profile):
    from candrive.config import PdoField, PdoMap
    bad = co.PdoCodec(PdoMap("too_big", 0x200,
                             [PdoField("target_velocity", 1, "i32"),
                              PdoField("target_velocity", 2, "i32"),
                              PdoField("controlword", 1, "u16")]), 1)
    with pytest.raises(ValueError, match="8바이트"):
        bad.pack({})


def test_statusword_state_decoding(profile):
    assert profile.state_of(0x0040) == "switch_on_disabled"
    assert profile.state_of(0x0021) == "ready_to_switch_on"
    assert profile.state_of(0x0023) == "switched_on"
    assert profile.state_of(0x0237) == "operation_enabled"
    assert profile.state_of(0x0008) == "fault"


def test_cob_id_map_is_unique(profile):
    hw = HardwareConfig.load()
    rx, tx = co.build_codecs(profile, hw.node_id)
    ids = [c.cob_id for c in rx] + [c.cob_id for c in tx] + [
        profile.cob["sdo_rx_base"] + hw.node_id,
        profile.cob["sdo_tx_base"] + hw.node_id,
        profile.cob["heartbeat_base"] + hw.node_id,
    ]
    assert len(ids) == len(set(ids)), f"COB-ID 충돌: {ids}"

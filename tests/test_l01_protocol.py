"""Driver packet parsing against the datasheet's own command/response examples (Figures 17-19, Table 15)."""
import pytest
from hypothesis import given, settings
from hypothesis import strategies as st

from goldenfleece.l01_radar_data_input import protocol as K


def test_datasheet_fig17_init():
    assert K.cmd_init(115200) == bytes.fromhex("49 4E 49 54 04 00 00 00 00 00 00 00")
    p = K.Kld7Parser().feed(bytes.fromhex("52 45 53 50 01 00 00 00 00"))
    assert p == [K.Packet(b"RESP", b"\x00")] and K.parse_resp(p[0].payload) == K.RespCode.OK


def test_datasheet_fig18_gnfd_and_tdat_layout():
    assert K.cmd_gnfd(0x08) == bytes.fromhex("47 4E 46 44 04 00 00 00 08 00 00 00")
    assert K.cmd_gnfd() == bytes.fromhex("47 4E 46 44 04 00 00 00 24 00 00 00")      # PDAT | DONE = 0x24
    pk = K.Kld7Parser().feed(bytes.fromhex("52 45 53 50 01 00 00 00 00") + bytes.fromhex("54 44 41 54 08 00 00 00 50 00 97 FF 2F 07 15 18"))
    assert [p.header for p in pk] == [b"RESP", b"TDAT"]
    # Table 15: the 8-byte target layout (TDAT is never requested; PDAT uses the same layout)
    t = K.parse_pdat(pk[1].payload)[0]
    assert t.distance_cm == 80 and t.speed_raw == -105 and t.angle_raw == 1839 and t.magnitude_raw == 6165


def test_datasheet_fig19_gbye_erratum():
    """Figure 19 prints the INIT header bytes for the GBYE example; the header is ASCII 'GBYE'."""
    assert K.cmd_gbye() == b"GBYE" + bytes(4)
    assert K.cmd_gbye() != bytes.fromhex("49 4E 49 54 00 00 00 00")


def test_rpst_pack_unpack_roundtrip_and_defaults():
    p = K.RadarParams("K-LD7_APP-RFB-0104")
    b = p.pack()
    assert len(b) == 42
    q = K.RadarParams.unpack(b)
    assert q == p
    r = p.with_params({"RRAI": 2, "RSPI": 3, "THOF": 30, "DEDI": 2})
    assert r.max_range == 2 and r.max_speed == 3 and not r.same_settings(p)
    assert r.diff(p) == {"max_speed": (3, 1), "max_range": (2, 1)}
    with pytest.raises(ValueError):
        p.with_params({"XXXX": 1})


def test_pdat_parse_and_done():
    payload = b"".join(K.PDAT_TARGET_STRUCT.pack(2000, -5400, 863, 5000) for _ in range(12))
    tg = K.parse_pdat(payload)
    assert len(tg) == 12 and tg[0].speed_raw == -5400 and tg[0].angle_raw == 863
    with pytest.raises(ValueError):
        K.parse_pdat(payload + b"\x00" * 8)          # 13 targets cannot happen: 96-byte maximum
    assert K.parse_done(b"\x39\x30\x00\x00") == 12345
    assert K.parse_pdat(b"") == ()


def test_parser_resync_and_split_delivery():
    good = K.build_command(b"DONE", b"\x07\x00\x00\x00")
    p = K.Kld7Parser()
    out = p.feed(b"\xff\x12junk" + good[:5])
    assert out == [] and p.n_resync_bytes >= 4          # slides only while >= 8 bytes are buffered
    out = p.feed(good[5:] + b"PDAT" + b"\x10\x00\x00\x00" + b"\x01" * 16)
    assert [x.header for x in out] == [b"DONE", b"PDAT"] and p.n_resync_bytes == 6
    bad_len = b"PDAT" + b"\x07\x00\x00\x00" + b"\x00" * 7     # 7 is not a multiple of 8
    out = p.feed(bad_len + good)
    assert [x.header for x in out] == [b"DONE"] and p.n_bad_length == 1


@given(st.binary(min_size=0, max_size=300))
@settings(max_examples=200)
def test_parser_never_raises_on_garbage(data):
    p = K.Kld7Parser()
    p.feed(data)
    p.feed(K.build_command(b"RESP", b"\x00"))
    assert p.pending() <= 2 * K.MAX_PAYLOAD + 8


def test_wire_costs_match_stage0():
    assert abs(K.header_transfer_s(921600) - 4.77e-5) < 1e-7
    assert abs(125 * K.BITS_PER_BYTE_8E1 / 115200 - 0.01194) < 1e-4

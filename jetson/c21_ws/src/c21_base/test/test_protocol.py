import struct

from c21_base import protocol as P


def test_velocity_frame():
    f = P.encode_velocity(0.1, -0.2, 0.5)
    assert f[:4] == bytes([0xAA, 0x55, 0x0B, 0x50])
    assert struct.unpack('>hhh', f[4:10]) == (100, -200, 500)
    assert f[10] == sum(f[:10]) & 0xFF
    assert len(f) == 11


def test_servo_frame_len():
    f = P.encode_servos([0.5], 120)
    assert f[2] == len(f) == 19 and f[3] == 0x90
    assert struct.unpack('>h', f[4:6])[0] == 500


def _fb(vx=0, battery=94):
    payload = struct.pack('>15h', 0, 0, 1672, 0, 0, 0, vx, 0, 0, 0, 0, 0, 0, 0, 0) + bytes([battery, 0])
    body = b'\xAA\x55' + payload
    return body + bytes([sum(body) & 0xFF, 0x7D])


def test_parser_resync():
    p = P.FrameParser()
    data = b'\x01\xAA\x02' + _fb(150) + _fb()[:10]
    out = p.feed(data)
    assert len(out) == 1 and abs(out[0].vel[0] - 0.15) < 1e-9 and out[0].battery_raw == 94
    out = p.feed(_fb()[10:])
    assert len(out) == 1


def test_bad_checksum_rejected():
    f = bytearray(_fb())
    f[34] ^= 0xFF
    assert P.parse_feedback(bytes(f)) is None

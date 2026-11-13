"""OpenRF4 (STM32F407) serial protocol for the C21 chassis.

Downlink frames: AA 55 <total_len> <id> <payload...> <sum8>
  sum8 = sum of every preceding byte & 0xFF. Firmware rejects total_len > 30.
Uplink frame (36 bytes): AA 55 <32-byte payload> <sum8 over first 34 bytes> 7D
"""
import struct
from dataclasses import dataclass, field
from typing import List, Optional

HEADER = b'\xAA\x55'
FEEDBACK_LEN = 36
FEEDBACK_TAIL = 0x7D

ID_VEL = 0x50
ID_IMU_CALIB = 0x51
ID_BEEP = 0x66
ID_KEY = 0x80
ID_IK = 0x90

# Vendor scaling (car_base.h): accel +-2g, gyro +-500 deg/s
ACCEL_RATIO = 1671.84
GYRO_RATIO = 0.00026644

INT16_MIN, INT16_MAX = -32768, 32767


def checksum(data: bytes) -> int:
    return sum(data) & 0xFF


def _i16(value: float) -> int:
    v = int(round(value))
    return max(INT16_MIN, min(INT16_MAX, v))


def _frame(frame_id: int, payload: bytes) -> bytes:
    body = HEADER + bytes([len(payload) + 5, frame_id]) + payload
    return body + bytes([checksum(body)])


def encode_velocity(vx: float, vy: float, wz: float) -> bytes:
    """vx, vy in m/s, wz in rad/s; sent as int16 big-endian x1000."""
    return _frame(ID_VEL, struct.pack('>hhh', _i16(vx * 1000), _i16(vy * 1000), _i16(wz * 1000)))


def encode_beep(times: int = 1, on_ms: int = 100, off_ms: int = 100) -> bytes:
    # Firmware divides the first field by 1000.
    return _frame(ID_BEEP, struct.pack('>hhh', _i16(times * 1000), _i16(on_ms), _i16(off_ms)))


def encode_servos(angles_rad: List[float], move_ms: int = 100) -> bytes:
    """Six servo angles (rad) with explicit move time (ID_IK frame)."""
    a = (list(angles_rad) + [0.0] * 6)[:6]
    payload = struct.pack('>6h', *[_i16(x * 1000) for x in a]) + struct.pack('>H', max(0, min(65535, int(move_ms))))
    return _frame(ID_IK, payload)


def encode_imu_calibration() -> bytes:
    return _frame(ID_IMU_CALIB, b'\x00')


@dataclass
class Feedback:
    accel: tuple          # m/s^2
    gyro: tuple           # rad/s
    vel: tuple            # vx, vy (m/s), wz (rad/s) as measured by MCU
    servos: List[float] = field(default_factory=list)  # raw int16 /1000
    battery_raw: int = 0
    ultrasonic_raw: int = 0


def parse_feedback(frame: bytes) -> Optional[Feedback]:
    if len(frame) != FEEDBACK_LEN or frame[:2] != HEADER or frame[35] != FEEDBACK_TAIL:
        return None
    if checksum(frame[:34]) != frame[34]:
        return None
    p = frame[2:34]
    v = struct.unpack('>15h', p[:30])
    return Feedback(
        accel=tuple(x / ACCEL_RATIO for x in v[0:3]),
        gyro=tuple(x * GYRO_RATIO for x in v[3:6]),
        vel=tuple(x / 1000.0 for x in v[6:9]),
        servos=[x / 1000.0 for x in v[9:15]],
        battery_raw=p[30],
        ultrasonic_raw=p[31],
    )


class FrameParser:
    """Incremental resynchronising parser for the uplink byte stream."""

    def __init__(self):
        self._buf = bytearray()
        self.good = 0
        self.bad = 0

    def feed(self, data: bytes) -> List[Feedback]:
        self._buf.extend(data)
        out = []
        while True:
            i = self._buf.find(HEADER)
            if i < 0:
                # keep a trailing 0xAA that may start the next header
                del self._buf[:-1]
                break
            if i:
                del self._buf[:i]
            if len(self._buf) < FEEDBACK_LEN:
                break
            fb = parse_feedback(bytes(self._buf[:FEEDBACK_LEN]))
            if fb is None:
                self.bad += 1
                del self._buf[:1]
                continue
            self.good += 1
            out.append(fb)
            del self._buf[:FEEDBACK_LEN]
        if len(self._buf) > 4096:
            del self._buf[:-FEEDBACK_LEN]
        return out

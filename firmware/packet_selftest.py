"""Self-test for the dual_imu_binary packet format (90 bytes, little-endian).

Run: python packet_selftest.py
"""
import struct

PACKET_FMT = "<2sHI10f10fH"  # sync, seq, t_us, IMU1(10 f32), IMU2(10 f32), crc
SYNC = b"\xAA\x55"


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def build_packet(seq: int, t_us: int, imu1, imu2) -> bytes:
    body = struct.pack("<HI10f10f", seq, t_us, *imu1, *imu2)
    return SYNC + body + struct.pack("<H", crc16_ccitt_false(body))


def parse_packet(pkt: bytes):
    if len(pkt) != 90 or pkt[:2] != SYNC:
        raise ValueError("bad length/sync")
    if crc16_ccitt_false(pkt[2:88]) != struct.unpack_from("<H", pkt, 88)[0]:
        raise ValueError("bad crc")
    f = struct.unpack(PACKET_FMT, pkt)
    return f[1], f[2], f[3:13], f[13:23]


def main():
    assert struct.calcsize(PACKET_FMT) == 90, struct.calcsize(PACKET_FMT)

    tv = crc16_ccitt_false(b"123456789")
    assert tv == 0x29B1, hex(tv)
    print(f"CRC test vector '123456789' -> 0x{tv:04X} OK")

    imu1 = [0.1, -0.2, 9.81, 0.01, -0.02, 0.03, 20.0, -5.0, 40.0, 25.5]
    imu2 = [-0.1, 0.2, 9.80, -0.01, 0.02, -0.03, 21.0, -6.0, 41.0, 26.25]
    pkt = build_packet(1234, 987654321, imu1, imu2)
    assert len(pkt) == 90, len(pkt)
    seq, t_us, a, b = parse_packet(pkt)
    assert seq == 1234 and t_us == 987654321
    assert all(abs(x - y) < 1e-6 for x, y in zip(a + b, imu1 + imu2))

    bad = bytearray(pkt)
    bad[10] ^= 0x01
    try:
        parse_packet(bytes(bad))
        raise AssertionError("corrupted packet not detected")
    except ValueError:
        pass

    print(f"Packet size {len(pkt)} OK, crc=0x{struct.unpack_from('<H', pkt, 88)[0]:04X}")
    print("Reference packet hex:", pkt.hex())
    print("ALL TESTS PASSED")


if __name__ == "__main__":
    main()

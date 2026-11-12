# Dual-IMU ESP32 firmware (binary stream)

Sketch: `dual_imu_binary/dual_imu_binary.ino`. This replaces `microcontroller_upload_code_arduinoIDE.txt`, which was the ASCII stream at 115200 baud and topped out at about 57 Hz.

## Wiring

All four sensors share one I²C bus at 3V3.

| ESP32  | Sensor boards |
|--------|---------------|
| GPIO21 | SDA           |
| GPIO22 | SCL           |
| 3V3    | VIN / 3Vo     |
| GND    | GND           |

| Sensor        | Role | I²C addr |
|---------------|------|----------|
| ISM330DHCX #1 | IMU1 | 0x6B     |
| LIS3MDL #1    | MAG1 | 0x1E     |
| ISM330DHCX #2 | IMU2 | 0x6A     |
| LIS3MDL #2    | MAG2 | 0x1C     |

## Required libraries (Arduino Library Manager)

- Adafruit LSM6DS (tested against 4.7.4; it provides `Adafruit_ISM330DHCX`)
- Adafruit LIS3MDL (tested against 1.2.5)
- Adafruit Unified Sensor
- Adafruit BusIO (installed as a dependency)

## Board settings

- Board: ESP32 Dev Module (`esp32:esp32:esp32`), Espressif ESP32 core 2.x or 3.x
- Upload speed: 921600 (lower it if uploads fail)
- Serial Monitor / host port: **921600 baud**, 8N1
- CPU 240 MHz, default flash and partition settings

## Sensor configuration

| Sensor     | Setting                                                            |
|------------|--------------------------------------------------------------------|
| ISM330DHCX | accel ±4 g, gyro ±250 dps, accel and gyro ODR 208 Hz               |
| LIS3MDL    | ultra-high-performance mode, 155 Hz (FAST_ODR), ±4 gauss, continuous |

## Compile-time options

| Define           | Default | Meaning                                                  |
|------------------|---------|----------------------------------------------------------|
| `SAMPLE_RATE_HZ` | 200     | Output rate. The schedule uses `micros()` and skips missed slots instead of bursting. |
| `OUTPUT_ASCII`   | 0       | 1 sends the legacy `R,<t_us>,<18 floats>` line instead (has no temperature). |
| `MAG_DECIMATION` | 1       | Reads the mags every Nth sample and repeats the last value in between. |

## Packet format (OUTPUT_ASCII = 0)

Each packet is 90 bytes, little-endian and packed.

| Offset | Size | Type      | Field  | Notes                                               |
|-------:|-----:|-----------|--------|-----------------------------------------------------|
| 0      | 2    | u8[2]     | sync   | `0xAA 0x55`                                         |
| 2      | 2    | uint16    | seq    | +1 per packet, wraps                                |
| 4      | 4    | uint32    | t_us   | `micros()` taken just before reading IMU1 (wraps about every 71.6 min) |
| 8      | 12   | float32×3 | IMU1 a | ax, ay, az [m/s²]                                   |
| 20     | 12   | float32×3 | IMU1 g | gx, gy, gz [rad/s]                                  |
| 32     | 12   | float32×3 | IMU1 m | mx, my, mz [µT]                                     |
| 44     | 4    | float32   | IMU1 T | temperature [°C]                                    |
| 48     | 40   | ...       | IMU2   | same layout as IMU1                                 |
| 88     | 2    | uint16    | crc    | CRC-16/CCITT-FALSE over bytes 2..87                 |

The CRC is poly 0x1021, init 0xFFFF, no reflection, xorout 0x0000. Its check value is `"123456789"` → `0x29B1`.
Python format string: `"<2sHI10f10fH"`.

To check the format and the CRC, run `python packet_selftest.py`. It also prints a reference packet in hex, and the firmware's C++ struct and CRC code produce that same packet byte for byte.

How to tell drops apart:
- A gap in `seq` means bytes were lost on the serial link.
- A `t_us` step much larger than 1/SAMPLE_RATE_HZ, with `seq` still contiguous, means the firmware skipped schedule slots.

## Bandwidth

- 90 B × 200 Hz = 18 000 B/s.
- 921600 baud ÷ 10 bits per byte (8N1) ≈ 92 160 B/s, so the link is about 20 % used.
- The headroom would allow about 1000 Hz.
- For comparison, the old ASCII stream sent about 200 B per line. At 115200 baud (11 520 B/s) that caps out at about 57 Hz.
- A 90-byte packet takes about 0.98 ms on the wire. The sketch raises the TX buffer to 1 KB, so `Serial.write()` never blocks.

## I²C time budget (400 kHz, 4 sensors, 200 Hz)

At 400 kHz each byte plus its ACK is 9 clocks, about 22.5 µs.

| Read                                       | Bytes on bus (approx.) | Time     |
|--------------------------------------------|------------------------|----------|
| ISM330DHCX `getEvent`: one 14-byte burst (temp + gyro + accel) | addr+reg, addr, 14 data ≈ 17 | ≈ 0.40 ms |
| LIS3MDL `getEvent`: one 6-byte burst       | addr+reg, addr, 6 data ≈ 9    | ≈ 0.21 ms |

- Raw bus time for 2 × IMU + 2 × mag is about 1.2 ms.
- The ESP32 Wire driver adds about 50–150 µs per transaction for start/stop and driver overhead. A realistic total is **about 1.5–2 ms per sample**.
- At 200 Hz the period is 5 ms, so the bus is about 30–40 % busy.
- About 400 Hz is the practical ceiling with all four sensors on one bus.
- `MAG_DECIMATION=2` cuts about 0.4 ms off every other sample.
- The sensor ODRs (208 Hz IMU, 155 Hz mag) are the real limit on fresh data. To go above 200 Hz, raise them too.

/*
 * dual_imu_binary.ino -- ESP32 dual-IMU streamer (binary framed output)
 *
 * WIRING (shared I2C bus, 3V3 logic)
 *   ESP32 GPIO21 -> SDA, ESP32 GPIO22 -> SCL, 3V3 -> VIN, GND -> GND
 *   IMU1: ISM330DHCX @ 0x6B  + LIS3MDL @ 0x1E
 *   IMU2: ISM330DHCX @ 0x6A  + LIS3MDL @ 0x1C
 *   I2C clock: 400 kHz. Serial: 921600 baud, 8N1.
 *
 * PACKET (OUTPUT_ASCII == 0), 90 bytes, little-endian, packed:
 *   off  size  field
 *   0    2     sync        0xAA 0x55
 *   2    2     uint16 seq  increments per emitted packet, wraps 65535 -> 0
 *   4    4     uint32 t_us micros() captured right before reading IMU1
 *   8    40    IMU1 block: float32 ax,ay,az [m/s^2], gx,gy,gz [rad/s],
 *                          mx,my,mz [uT], temp [degC]
 *   48   40    IMU2 block: same layout
 *   88   2     uint16 crc  CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF,
 *                          no reflection, xorout 0x0000) over bytes 2..87
 *
 * If the loop falls behind, whole sample slots are skipped (no bursting):
 * seq stays contiguous per packet, so gaps in t_us (not seq) indicate skipped
 * slots, while gaps in seq indicate bytes lost on the serial link.
 *
 * OUTPUT_ASCII == 1 emits the legacy line instead:
 *   "R,<t_us>,ax1,ay1,az1,gx1,gy1,gz1,mx1,my1,mz1,ax2,...,mz2\n" (18 floats)
 *
 * Library enum names verified against Adafruit_LSM6DS 4.7.4 and
 * Adafruit_LIS3MDL 1.2.5 headers:
 *   LSM6DS_ACCEL_RANGE_4_G, LSM6DS_GYRO_RANGE_250_DPS (there is no
 *   ISM330DHCX_GYRO_RANGE_250_DPS; only the 4000 dps entry has an ISM330DHCX_
 *   prefix), LSM6DS_RATE_208_HZ, LIS3MDL_DATARATE_155_HZ,
 *   LIS3MDL_ULTRAHIGHMODE, LIS3MDL_RANGE_4_GAUSS, LIS3MDL_CONTINUOUSMODE.
 */

#include <stddef.h>
#include <Wire.h>
#include <Adafruit_ISM330DHCX.h>
#include <Adafruit_LIS3MDL.h>

// ----------------------------- configuration -----------------------------
#ifndef OUTPUT_ASCII
#define OUTPUT_ASCII 0          // 1 = legacy "R,..." ASCII line
#endif

#ifndef SAMPLE_RATE_HZ
#define SAMPLE_RATE_HZ 200
#endif

// Read the magnetometers only every MAG_DECIMATION-th sample and repeat the
// last value in between. The LIS3MDL in UHP mode only updates at 155 Hz, so
// at 200 Hz some values repeat anyway; 2 halves mag bus traffic (~100 Hz
// fresh mag data). 1 = read every sample.
#ifndef MAG_DECIMATION
#define MAG_DECIMATION 1
#endif

#define SERIAL_BAUD 921600
#define I2C_SDA 21
#define I2C_SCL 22
#define I2C_CLOCK_HZ 400000

static const uint32_t SAMPLE_PERIOD_US = 1000000UL / SAMPLE_RATE_HZ;

// ------------------------------- packet ----------------------------------
struct __attribute__((packed)) ImuBlock {
  float ax, ay, az;   // m/s^2
  float gx, gy, gz;   // rad/s
  float mx, my, mz;   // uT
  float temp;         // degC
};

struct __attribute__((packed)) Packet {
  uint8_t  sync0;     // 0xAA
  uint8_t  sync1;     // 0x55
  uint16_t seq;
  uint32_t t_us;
  ImuBlock imu1;
  ImuBlock imu2;
  uint16_t crc;
};

static_assert(sizeof(ImuBlock) == 40, "ImuBlock must be 40 bytes");
static_assert(sizeof(Packet) == 90, "Packet must be 90 bytes");
static_assert(offsetof(Packet, crc) == 88, "crc must be at offset 88");

// ------------------------------- sensors ---------------------------------
Adafruit_ISM330DHCX imu1;
Adafruit_ISM330DHCX imu2;
Adafruit_LIS3MDL mag1;
Adafruit_LIS3MDL mag2;

static uint32_t nextSample = 0;
static uint16_t seq = 0;
static uint32_t sampleCount = 0;
static sensors_event_t m1, m2;  // persist so values repeat when decimated

// CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, xorout 0.
static uint16_t crc16_ccitt_false(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t b = 0; b < 8; b++) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

static void halt(const char *msg) {
  Serial.println(msg);
  while (true) {
    delay(1000);
  }
}

static void configureImu(Adafruit_ISM330DHCX &imu) {
  imu.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
  imu.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  imu.setAccelDataRate(LSM6DS_RATE_208_HZ);
  imu.setGyroDataRate(LSM6DS_RATE_208_HZ);
}

static void configureMag(Adafruit_LIS3MDL &mag) {
  mag.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
  mag.setDataRate(LIS3MDL_DATARATE_155_HZ);  // FAST_ODR + UHP (highest UHP rate)
  mag.setRange(LIS3MDL_RANGE_4_GAUSS);
  mag.setOperationMode(LIS3MDL_CONTINUOUSMODE);
}

static inline void fillBlock(ImuBlock &b, const sensors_event_t &a,
                             const sensors_event_t &g, const sensors_event_t &m,
                             const sensors_event_t &t) {
  b.ax = a.acceleration.x; b.ay = a.acceleration.y; b.az = a.acceleration.z;
  b.gx = g.gyro.x;         b.gy = g.gyro.y;         b.gz = g.gyro.z;
  b.mx = m.magnetic.x;     b.my = m.magnetic.y;     b.mz = m.magnetic.z;
  b.temp = t.temperature;
}

#if OUTPUT_ASCII
static void printTriple(float x, float y, float z) {
  Serial.print(','); Serial.print(x, 6);
  Serial.print(','); Serial.print(y, 6);
  Serial.print(','); Serial.print(z, 6);
}
#endif

void setup() {
  // Larger TX buffer so Serial.write() of a packet never blocks the loop.
  Serial.setTxBufferSize(1024);
  Serial.begin(SERIAL_BAUD);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_CLOCK_HZ);

  if (!imu1.begin_I2C(0x6B, &Wire)) halt("IMU1 ERROR (ISM330DHCX @0x6B)");
  if (!mag1.begin_I2C(0x1E, &Wire)) halt("MAG1 ERROR (LIS3MDL @0x1E)");
  if (!imu2.begin_I2C(0x6A, &Wire)) halt("IMU2 ERROR (ISM330DHCX @0x6A)");
  if (!mag2.begin_I2C(0x1C, &Wire)) halt("MAG2 ERROR (LIS3MDL @0x1C)");

  configureImu(imu1);
  configureImu(imu2);
  configureMag(mag1);
  configureMag(mag2);

  // Re-assert the bus clock in case a library begin() re-initialised Wire.
  Wire.setClock(I2C_CLOCK_HZ);

  // Prime mag values so the first packets are valid even when decimating.
  mag1.getEvent(&m1);
  mag2.getEvent(&m2);

  nextSample = micros();
}

void loop() {
  uint32_t now = micros();
  if ((int32_t)(now - nextSample) < 0) return;

  // Advance the schedule. If we are more than one period late, skip the
  // missed slots (keeping phase) instead of bursting to catch up.
  nextSample += SAMPLE_PERIOD_US;
  if ((int32_t)(now - nextSample) >= 0) {
    uint32_t late = now - nextSample;
    nextSample += (late / SAMPLE_PERIOD_US + 1) * SAMPLE_PERIOD_US;
  }

  sensors_event_t a1, g1, t1, a2, g2, t2;

  const uint32_t t_us = micros();  // right before reading IMU1
  imu1.getEvent(&a1, &g1, &t1);
  imu2.getEvent(&a2, &g2, &t2);
  if ((sampleCount % MAG_DECIMATION) == 0) {
    mag1.getEvent(&m1);
    mag2.getEvent(&m2);
  }
  sampleCount++;

#if OUTPUT_ASCII
  Serial.print("R,");
  Serial.print(t_us);
  printTriple(a1.acceleration.x, a1.acceleration.y, a1.acceleration.z);
  printTriple(g1.gyro.x, g1.gyro.y, g1.gyro.z);
  printTriple(m1.magnetic.x, m1.magnetic.y, m1.magnetic.z);
  printTriple(a2.acceleration.x, a2.acceleration.y, a2.acceleration.z);
  printTriple(g2.gyro.x, g2.gyro.y, g2.gyro.z);
  printTriple(m2.magnetic.x, m2.magnetic.y, m2.magnetic.z);
  Serial.println();
#else
  Packet pkt;
  pkt.sync0 = 0xAA;
  pkt.sync1 = 0x55;
  pkt.seq = seq++;
  pkt.t_us = t_us;
  fillBlock(pkt.imu1, a1, g1, m1, t1);
  fillBlock(pkt.imu2, a2, g2, m2, t2);
  const uint8_t *raw = reinterpret_cast<const uint8_t *>(&pkt);
  pkt.crc = crc16_ccitt_false(raw + 2, offsetof(Packet, crc) - 2);  // bytes 2..87
  Serial.write(raw, sizeof(pkt));
#endif
}

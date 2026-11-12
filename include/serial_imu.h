#pragma once

#include <string>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <vector>
#include <limits>

#include "math3d.h"

struct IMUSample
{
    float ax;
    float ay;
    float az;

    float gx;
    float gy;
    float gz;

    float mx;
    float my;
    float mz;

    // Die temperature, degC. NaN / hasTemp = false when the
    // stream does not carry it (ASCII mode).
    float temp = std::numeric_limits<float>::quiet_NaN();
    bool hasTemp = false;
};

struct RawIMUPacket
{
    uint32_t timestamp;

    // Sample counter. In ASCII mode this is a host-side
    // counter and hasSeq = false.
    uint16_t seq = 0;
    bool hasSeq = false;

    IMUSample imu1;
    IMUSample imu2;
};

//--------------------------------------------------
// Link statistics
//--------------------------------------------------

struct SerialIMUStats
{
    uint64_t binaryPackets = 0;
    uint64_t asciiPackets  = 0;
    uint64_t replayPackets = 0;   // samples played back from a CSV

    uint64_t crcErrors     = 0;   // binary frames rejected by CRC
    uint64_t parseErrors   = 0;   // malformed ASCII lines
    uint64_t droppedSamples = 0;  // packets lost on the link (seq gaps)
    uint64_t skippedSlots  = 0;   // firmware skipped slots (t_us step, no seq gap)

    uint64_t bytesRead     = 0;
};

class SerialIMU
{
public:
    // Binary frame layout (little-endian, packed):
    // [AA 55][seq u16][t_us u32][IMU1 10xf32][IMU2 10xf32][crc u16]
    static constexpr size_t BINARY_PACKET_SIZE = 90;

    ~SerialIMU();

    bool open(const std::string& portName, int baudRate);
    void close();

    // Plays back a raw_log.csv recording instead of a serial
    // port, paced by its t_us column (scaled by speed) and
    // looped at end of file. No hardware needed.
    bool openReplay(const std::string& path, float speed = 1.0f);
    bool isReplay() const;

    // Reads every pending byte from the port and parses all
    // complete packets (binary or ASCII, auto-detected) into
    // the queue. Returns true if any packet is queued.
    bool update();

    // Pops the oldest queued packet. Call in a loop until it
    // returns false to process every received sample.
    bool nextPacket(RawIMUPacket& out);

    size_t queuedPackets() const;

    const RawIMUPacket& packet() const;

    const SerialIMUStats& stats() const;
    void resetStats();

    //--------------------------------------------------
    // Raw CSV log
    //--------------------------------------------------

    bool startRawLog(const std::string& path);
    void stopRawLog();
    bool isRawLogging() const;

    static uint16_t crc16CcittFalse(
        const uint8_t* data,
        size_t length
    );

private:
    void parseBuffer();

    bool parseBinary(const uint8_t* data, RawIMUPacket& out);
    bool parseAscii(const std::string& line, RawIMUPacket& out);

    void pushPacket(const RawIMUPacket& packet);
    void logPacket(const RawIMUPacket& packet);

    void updateReplay();
    bool readReplayLine(RawIMUPacket& out);

private:
    void* m_handle = nullptr;

    RawIMUPacket m_packet = {};

    std::vector<uint8_t> m_rx;
    std::deque<RawIMUPacket> m_queue;

    SerialIMUStats m_stats;

    uint16_t m_lastSeq = 0;
    bool m_hasLastSeq = false;

    uint32_t m_lastSeqTimestamp = 0;

    // Running estimate of the firmware sample period, us.
    float m_nominalPeriodUs = 0.0f;

    uint16_t m_asciiSeq = 0;

    std::FILE* m_rawLog = nullptr;

    //--------------------------------------------------
    // Replay state
    //--------------------------------------------------

    std::FILE* m_replay = nullptr;
    long m_replayDataStart = 0;     // file offset after header
    float m_replaySpeed = 1.0f;
    uint64_t m_replayStartMs = 0;   // wall clock at start
    uint64_t m_replayElapsedUs = 0; // recording time emitted so far
    uint32_t m_replayPrevUs = 0;    // previous raw t_us
    uint32_t m_replayOffsetUs = 0;  // keeps t_us monotonic across loops
    uint32_t m_replayLastOutUs = 0;
    bool m_replayHasPrev = false;
    bool m_replayHasPending = false;
    RawIMUPacket m_replayPending = {};
};

#include "serial_imu.h"

#include <windows.h>
#include <iostream>
#include <cmath>
#include <cstring>
#include <cstdlib>

//--------------------------------------------------
// Protocol constants
//--------------------------------------------------

static const uint8_t SYNC0 = 0xAA;
static const uint8_t SYNC1 = 0x55;

// Longest ASCII "R,..." line accepted before giving up
// on a candidate and resyncing.
static const size_t MAX_ASCII_LINE = 512;

// Upper bound for queued packets if nobody consumes them.
static const size_t MAX_QUEUED_PACKETS = 20000;

//--------------------------------------------------
// Little-endian helpers
//--------------------------------------------------

static uint16_t readU16(const uint8_t* p)
{
    return static_cast<uint16_t>(p[0] | (p[1] << 8));
}

static uint32_t readU32(const uint8_t* p)
{
    return  static_cast<uint32_t>(p[0])
         | (static_cast<uint32_t>(p[1]) << 8)
         | (static_cast<uint32_t>(p[2]) << 16)
         | (static_cast<uint32_t>(p[3]) << 24);
}

static float readF32(const uint8_t* p)
{
    uint32_t bits = readU32(p);
    float value;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

static const uint8_t* readSampleBlock(
    const uint8_t* p,
    IMUSample& s
)
{
    s.ax = readF32(p +  0);
    s.ay = readF32(p +  4);
    s.az = readF32(p +  8);

    s.gx = readF32(p + 12);
    s.gy = readF32(p + 16);
    s.gz = readF32(p + 20);

    s.mx = readF32(p + 24);
    s.my = readF32(p + 28);
    s.mz = readF32(p + 32);

    s.temp = readF32(p + 36);
    s.hasTemp = true;

    return p + 40;
}

//--------------------------------------------------
// Open / Close
//--------------------------------------------------

SerialIMU::~SerialIMU()
{
    close();
}

bool SerialIMU::open(const std::string& portName, int baudRate)
{
    std::string fullPortName = "\\\\.\\" + portName;

    HANDLE handle = CreateFileA(
        fullPortName.c_str(),
        GENERIC_READ,
        0,
        nullptr,
        OPEN_EXISTING,
        0,
        nullptr
    );

    if (handle == INVALID_HANDLE_VALUE)
        return false;

    // Larger driver buffer for high baud rates.
    SetupComm(handle, 1 << 16, 1 << 12);

    DCB dcb = {};
    dcb.DCBlength = sizeof(dcb);

    if (!GetCommState(handle, &dcb))
    {
        CloseHandle(handle);
        return false;
    }

    dcb.BaudRate = static_cast<DWORD>(baudRate);
    dcb.ByteSize = 8;
    dcb.StopBits = ONESTOPBIT;
    dcb.Parity   = NOPARITY;
    dcb.fBinary  = TRUE;
    dcb.fParity  = FALSE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fInX     = FALSE;
    dcb.fOutX    = FALSE;

    if (!SetCommState(handle, &dcb))
    {
        CloseHandle(handle);
        return false;
    }

    // Non-blocking reads: return immediately with whatever
    // is already in the driver buffer.
    COMMTIMEOUTS timeouts = {};
    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutConstant = 0;
    timeouts.ReadTotalTimeoutMultiplier = 0;

    SetCommTimeouts(handle, &timeouts);

    PurgeComm(handle, PURGE_RXCLEAR);

    m_handle = handle;

    m_rx.clear();
    m_queue.clear();
    m_hasLastSeq = false;

    return true;
}

void SerialIMU::close()
{
    if (m_handle)
    {
        CloseHandle((HANDLE)m_handle);
        m_handle = nullptr;
    }

    if (m_replay)
    {
        std::fclose(m_replay);
        m_replay = nullptr;
    }

    stopRawLog();
}

//--------------------------------------------------
// Replay (raw_log.csv playback)
//--------------------------------------------------

bool SerialIMU::openReplay(const std::string& path, float speed)
{
    std::FILE* file = std::fopen(path.c_str(), "r");

    if (!file)
        return false;

    // Skip the header line.
    char header[1024];

    if (!std::fgets(header, sizeof(header), file))
    {
        std::fclose(file);
        return false;
    }

    m_replay = file;
    m_replayDataStart = std::ftell(file);
    m_replaySpeed = speed > 0.0f ? speed : 1.0f;
    m_replayStartMs = GetTickCount64();
    m_replayElapsedUs = 0;
    m_replayHasPrev = false;
    m_replayHasPending = false;
    m_replayOffsetUs = 0;

    m_queue.clear();
    m_hasLastSeq = false;

    return true;
}

bool SerialIMU::isReplay() const
{
    return m_replay != nullptr;
}

bool SerialIMU::readReplayLine(RawIMUPacket& out)
{
    char line[1024];

    for (int attempt = 0; attempt < 2; ++attempt)
    {
        while (std::fgets(line, sizeof(line), m_replay))
        {
            // seq,t_us, 10 values IMU1, 10 values IMU2
            float v[20];
            char* p = line;
            char* end = nullptr;

            unsigned long seq = std::strtoul(p, &end, 10);
            if (end == p || *end != ',') continue;
            p = end + 1;

            unsigned long t = std::strtoul(p, &end, 10);
            if (end == p || *end != ',') continue;
            p = end + 1;

            bool ok = true;

            for (int i = 0; i < 20; ++i)
            {
                v[i] = std::strtof(p, &end);

                if (end == p) { ok = false; break; }

                p = (*end == ',') ? end + 1 : end;
            }

            if (!ok)
                continue;

            out = {};
            out.seq = static_cast<uint16_t>(seq);
            out.hasSeq = true;
            out.timestamp = static_cast<uint32_t>(t);

            IMUSample* imus[2] = { &out.imu1, &out.imu2 };

            for (int k = 0; k < 2; ++k)
            {
                const float* s = v + k * 10;
                IMUSample& d = *imus[k];

                d.ax = s[0]; d.ay = s[1]; d.az = s[2];
                d.gx = s[3]; d.gy = s[4]; d.gz = s[5];
                d.mx = s[6]; d.my = s[7]; d.mz = s[8];
                d.temp = s[9];
                d.hasTemp = !std::isnan(s[9]);
            }

            return true;
        }

        // End of file: loop back to the first sample.
        std::clearerr(m_replay);
        std::fseek(m_replay, m_replayDataStart, SEEK_SET);

        m_replayHasPrev = false;
        m_hasLastSeq = false;
    }

    return false;
}

void SerialIMU::updateReplay()
{
    const uint64_t wallUs =
        (GetTickCount64() - m_replayStartMs) * 1000ull;

    const uint64_t targetUs =
        static_cast<uint64_t>(wallUs * m_replaySpeed);

    // Cap work per frame so a stall can't freeze the UI.
    for (int i = 0; i < 20000; ++i)
    {
        if (!m_replayHasPending)
        {
            RawIMUPacket packet;

            if (!readReplayLine(packet))
                return;

            const uint32_t raw = packet.timestamp;

            if (m_replayHasPrev)
            {
                m_replayElapsedUs += raw - m_replayPrevUs;
            }
            else if (m_replayElapsedUs > 0)
            {
                // Looped: continue timestamps one period later.
                m_replayOffsetUs =
                    (m_replayLastOutUs + 5000u) - raw;
            }

            m_replayPrevUs = raw;
            m_replayHasPrev = true;

            packet.timestamp = raw + m_replayOffsetUs;
            m_replayPending = packet;
            m_replayHasPending = true;
        }

        if (m_replayElapsedUs > targetUs)
            return;

        m_replayLastOutUs = m_replayPending.timestamp;
        ++m_stats.replayPackets;
        pushPacket(m_replayPending);
        m_replayHasPending = false;
    }
}

//--------------------------------------------------
// Update
//--------------------------------------------------

bool SerialIMU::update()
{
    if (m_replay)
    {
        updateReplay();
    }
    else if (m_handle)
    {
        uint8_t buffer[4096];

        // Drain everything available this frame.
        for (int i = 0; i < 64; ++i)
        {
            DWORD bytesRead = 0;

            BOOL ok = ReadFile(
                (HANDLE)m_handle,
                buffer,
                sizeof(buffer),
                &bytesRead,
                nullptr
            );

            if (!ok || bytesRead == 0)
                break;

            m_rx.insert(m_rx.end(), buffer, buffer + bytesRead);
            m_stats.bytesRead += bytesRead;

            if (bytesRead < sizeof(buffer))
                break;
        }

        parseBuffer();
    }

    return !m_queue.empty();
}

bool SerialIMU::nextPacket(RawIMUPacket& out)
{
    if (m_queue.empty())
        return false;

    m_packet = m_queue.front();
    m_queue.pop_front();

    out = m_packet;

    return true;
}

size_t SerialIMU::queuedPackets() const
{
    return m_queue.size();
}

//--------------------------------------------------
// Parsing
//--------------------------------------------------

void SerialIMU::parseBuffer()
{
    size_t pos = 0;
    const size_t size = m_rx.size();

    while (pos < size)
    {
        const uint8_t* p = m_rx.data() + pos;
        size_t remaining = size - pos;

        //--------------------------------
        // Binary frame
        //--------------------------------

        if (p[0] == SYNC0)
        {
            if (remaining < 2)
                break;

            if (p[1] != SYNC1)
            {
                ++pos;
                continue;
            }

            if (remaining < BINARY_PACKET_SIZE)
                break;

            RawIMUPacket packet = {};

            if (parseBinary(p, packet))
            {
                pushPacket(packet);
                pos += BINARY_PACKET_SIZE;
            }
            else
            {
                // CRC failure: resync on next sync word.
                ++m_stats.crcErrors;
                ++pos;
            }

            continue;
        }

        //--------------------------------
        // ASCII line
        //--------------------------------

        if (p[0] == 'R')
        {
            size_t limit = remaining < MAX_ASCII_LINE
                ? remaining
                : MAX_ASCII_LINE;

            const void* nl = std::memchr(p, '\n', limit);

            if (!nl)
            {
                if (remaining < MAX_ASCII_LINE)
                    break;          // wait for rest of line

                ++m_stats.parseErrors;
                ++pos;
                continue;
            }

            size_t length =
                static_cast<const uint8_t*>(nl) - p;

            std::string line(
                reinterpret_cast<const char*>(p),
                length
            );

            RawIMUPacket packet = {};

            if (parseAscii(line, packet))
            {
                pushPacket(packet);
                pos += length + 1;
            }
            else
            {
                // Could be binary payload that happens to
                // start with 'R'; advance one byte only.
                ++m_stats.parseErrors;
                ++pos;
            }

            continue;
        }

        //--------------------------------
        // Skip to next candidate
        //--------------------------------

        ++pos;

        while (pos < size &&
               m_rx[pos] != SYNC0 &&
               m_rx[pos] != 'R')
        {
            ++pos;
        }
    }

    m_rx.erase(m_rx.begin(), m_rx.begin() + pos);
}

bool SerialIMU::parseBinary(const uint8_t* data, RawIMUPacket& out)
{
    // CRC over seq .. end of IMU2 block (bytes 2..87).
    uint16_t crc =
        crc16CcittFalse(data + 2, BINARY_PACKET_SIZE - 4);

    uint16_t expected =
        readU16(data + BINARY_PACKET_SIZE - 2);

    if (crc != expected)
        return false;

    out.seq = readU16(data + 2);
    out.hasSeq = true;
    out.timestamp = readU32(data + 4);

    const uint8_t* p = data + 8;

    p = readSampleBlock(p, out.imu1);
    p = readSampleBlock(p, out.imu2);

    ++m_stats.binaryPackets;

    return true;
}

bool SerialIMU::parseAscii(const std::string& line, RawIMUPacket& out)
{
    // Format: R,t,ax1..mz1,ax2..mz2[,temp1,temp2]
    std::vector<std::string> fields;

    size_t start = 0;

    while (true)
    {
        size_t comma = line.find(',', start);

        if (comma == std::string::npos)
        {
            fields.push_back(line.substr(start));
            break;
        }

        fields.push_back(line.substr(start, comma - start));
        start = comma + 1;
    }

    if (fields.size() != 20 && fields.size() != 22)
        return false;

    if (fields[0] != "R")
        return false;

    char* end = nullptr;

    unsigned long timestamp =
        std::strtoul(fields[1].c_str(), &end, 10);

    if (end == fields[1].c_str())
        return false;

    float values[20] = {};

    for (size_t i = 2; i < fields.size(); ++i)
    {
        const char* text = fields[i].c_str();

        values[i - 2] = std::strtof(text, &end);

        if (end == text)
            return false;

        // Allow trailing whitespace / '\r' only.
        while (*end == ' ' || *end == '\r' || *end == '\t')
            ++end;

        if (*end != '\0')
            return false;
    }

    out.timestamp = static_cast<uint32_t>(timestamp);

    out.seq = m_asciiSeq++;
    out.hasSeq = false;

    IMUSample* imus[2] = { &out.imu1, &out.imu2 };

    for (int k = 0; k < 2; ++k)
    {
        const float* v = values + k * 9;
        IMUSample& s = *imus[k];

        s.ax = v[0]; s.ay = v[1]; s.az = v[2];
        s.gx = v[3]; s.gy = v[4]; s.gz = v[5];
        s.mx = v[6]; s.my = v[7]; s.mz = v[8];

        if (fields.size() == 22)
        {
            s.temp = values[18 + k];
            s.hasTemp = true;
        }
        else
        {
            s.temp = std::numeric_limits<float>::quiet_NaN();
            s.hasTemp = false;
        }
    }

    ++m_stats.asciiPackets;

    return true;
}

void SerialIMU::pushPacket(const RawIMUPacket& packet)
{
    //--------------------------------
    // Link drops vs. skipped slots
    //--------------------------------
    // Firmware increments seq once per packet SENT:
    //  - seq gap               -> packets lost on the link
    //  - no seq gap, large t_us -> firmware skipped schedule slots
    // dt itself is always computed from t_us (pipeline).

    if (packet.hasSeq)
    {
        if (m_hasLastSeq)
        {
            uint16_t gap =
                static_cast<uint16_t>(packet.seq - m_lastSeq);

            uint32_t deltaUs =
                packet.timestamp - m_lastSeqTimestamp;

            // Large "gaps" are treated as a device restart.
            if (gap > 1 && gap < 0x8000)
            {
                m_stats.droppedSamples += gap - 1;
            }
            else if (gap == 1 && deltaUs > 0)
            {
                float delta = static_cast<float>(deltaUs);

                if (m_nominalPeriodUs <= 0.0f)
                {
                    m_nominalPeriodUs = delta;
                }
                else if (delta > 1.5f * m_nominalPeriodUs)
                {
                    // Only count plausible stalls (< 1 s).
                    if (deltaUs < 1000000)
                    {
                        float slots =
                            std::round(delta / m_nominalPeriodUs);

                        if (slots > 1.0f)
                            m_stats.skippedSlots +=
                                static_cast<uint64_t>(slots) - 1;
                    }
                }
                else
                {
                    // Track the nominal period from contiguous samples.
                    m_nominalPeriodUs +=
                        0.01f * (delta - m_nominalPeriodUs);
                }
            }
        }

        m_lastSeq = packet.seq;
        m_lastSeqTimestamp = packet.timestamp;
        m_hasLastSeq = true;
    }

    logPacket(packet);

    m_queue.push_back(packet);

    while (m_queue.size() > MAX_QUEUED_PACKETS)
        m_queue.pop_front();
}

//--------------------------------------------------
// CRC-16/CCITT-FALSE
// poly 0x1021, init 0xFFFF, no reflection, xorout 0
//--------------------------------------------------

uint16_t SerialIMU::crc16CcittFalse(
    const uint8_t* data,
    size_t length
)
{
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < length; ++i)
    {
        crc ^= static_cast<uint16_t>(data[i]) << 8;

        for (int bit = 0; bit < 8; ++bit)
        {
            if (crc & 0x8000)
                crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
            else
                crc = static_cast<uint16_t>(crc << 1);
        }
    }

    return crc;
}

//--------------------------------------------------
// Raw CSV log
//--------------------------------------------------

bool SerialIMU::startRawLog(const std::string& path)
{
    stopRawLog();

    m_rawLog = std::fopen(path.c_str(), "w");

    if (!m_rawLog)
    {
        std::cerr << "Failed to open raw log: " << path << "\n";
        return false;
    }

    std::fprintf(
        m_rawLog,
        "seq,t_us,"
        "ax1,ay1,az1,gx1,gy1,gz1,mx1,my1,mz1,temp1,"
        "ax2,ay2,az2,gx2,gy2,gz2,mx2,my2,mz2,temp2\n"
    );

    return true;
}

void SerialIMU::stopRawLog()
{
    if (m_rawLog)
    {
        std::fclose(m_rawLog);
        m_rawLog = nullptr;
    }
}

bool SerialIMU::isRawLogging() const
{
    return m_rawLog != nullptr;
}

void SerialIMU::logPacket(const RawIMUPacket& packet)
{
    if (!m_rawLog)
        return;

    const IMUSample& a = packet.imu1;
    const IMUSample& b = packet.imu2;

    std::fprintf(
        m_rawLog,
        "%u,%lu,",
        static_cast<unsigned>(packet.seq),
        static_cast<unsigned long>(packet.timestamp)
    );

    const IMUSample* imus[2] = { &a, &b };

    for (int k = 0; k < 2; ++k)
    {
        const IMUSample& s = *imus[k];

        std::fprintf(
            m_rawLog,
            "%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,",
            s.ax, s.ay, s.az, s.gx, s.gy, s.gz, s.mx, s.my, s.mz
        );

        // Explicit "nan" keeps the CSV portable across CRTs.
        if (s.hasTemp && !std::isnan(s.temp))
            std::fprintf(m_rawLog, "%.7g", s.temp);
        else
            std::fprintf(m_rawLog, "nan");

        std::fputc(k == 0 ? ',' : '\n', m_rawLog);
    }
}

//--------------------------------------------------
// Accessors
//--------------------------------------------------

const RawIMUPacket& SerialIMU::packet() const
{
    return m_packet;
}

const SerialIMUStats& SerialIMU::stats() const
{
    return m_stats;
}

void SerialIMU::resetStats()
{
    m_stats = SerialIMUStats();
    m_hasLastSeq = false;
}

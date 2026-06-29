#pragma once

#include <string>
#include <cstdint>

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
};

struct RawIMUPacket
{
    uint32_t timestamp;

    IMUSample imu1;
    IMUSample imu2;
};

class SerialIMU
{
public:
    bool open(const std::string& portName, int baudRate);
    void close();

    bool update();

    const RawIMUPacket& packet() const;

private:
    void* m_handle = nullptr;

    RawIMUPacket m_packet;
};
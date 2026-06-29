#include "serial_imu.h"

#include <windows.h>
#include <sstream>
#include <iostream>
#include <cmath>

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

    DCB dcb = {};
    dcb.DCBlength = sizeof(dcb);

    if (!GetCommState(handle, &dcb))
        return false;

    dcb.BaudRate = baudRate;
    dcb.ByteSize = 8;
    dcb.StopBits = ONESTOPBIT;
    dcb.Parity   = NOPARITY;

    if (!SetCommState(handle, &dcb))
        return false;

    COMMTIMEOUTS timeouts = {};
    timeouts.ReadIntervalTimeout = 1;
    timeouts.ReadTotalTimeoutConstant = 1;
    timeouts.ReadTotalTimeoutMultiplier = 1;

    SetCommTimeouts(handle, &timeouts);

    m_handle = handle;

    return true;
}

void SerialIMU::close()
{
    if (m_handle)
    {
        CloseHandle((HANDLE)m_handle);
        m_handle = nullptr;
    }
}

bool SerialIMU::update()
{
    if (!m_handle)
        return false;

    char buffer[256];
    DWORD bytesRead = 0;

    BOOL ok = ReadFile(
        (HANDLE)m_handle,
        buffer,
        sizeof(buffer) - 1,
        &bytesRead,
        nullptr
    );

    if (!ok || bytesRead == 0)
        return false;

    buffer[bytesRead] = '\0';

    static std::string pending;
    pending += buffer;

    size_t newline = pending.rfind('\n');

    if (newline == std::string::npos)
        return false;

    size_t previous = pending.rfind('\n', newline - 1);

    std::string line;

    if (previous == std::string::npos)
    {
        line = pending.substr(0, newline);
    }
    else
    {
        line = pending.substr(previous + 1,
                            newline - previous - 1);
    }

    pending.erase(0, newline + 1);

    std::stringstream ss(line);

    char prefix;
    char comma;

    if (!(ss >> prefix))
        return false;

    if (prefix != 'R')
        return false;

    ss >> comma;

    if (comma != ',')
        return false;

    if (!(ss >>

        m_packet.timestamp >>

        comma >>

        m_packet.imu1.ax >>
        comma >>
        m_packet.imu1.ay >>
        comma >>
        m_packet.imu1.az >>

        comma >>
        m_packet.imu1.gx >>
        comma >>
        m_packet.imu1.gy >>
        comma >>
        m_packet.imu1.gz >>

        comma >>
        m_packet.imu1.mx >>
        comma >>
        m_packet.imu1.my >>
        comma >>
        m_packet.imu1.mz >>

        comma >>
        m_packet.imu2.ax >>
        comma >>
        m_packet.imu2.ay >>
        comma >>
        m_packet.imu2.az >>

        comma >>
        m_packet.imu2.gx >>
        comma >>
        m_packet.imu2.gy >>
        comma >>
        m_packet.imu2.gz >>

        comma >>
        m_packet.imu2.mx >>
        comma >>
        m_packet.imu2.my >>
        comma >>
        m_packet.imu2.mz))
    {
        return false;
    }

    return true;    

}

const RawIMUPacket& SerialIMU::packet() const
{
    return m_packet;
}
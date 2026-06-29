#pragma once

#include <string>
#include <vector>

#include "serial_imu.h"
#include "quaternion.h"
#include "filter/accelerometer_filter.h"
#include "filter/complementary_filter.h"
#include "filter/madgwick_filter.h"
#include "filter/mahony_filter.h"
#include "filter/mekf_filter.h"



enum class StreamType
{
    RawIMU,
    IMU,
    Quaternion,
    Euler,
    Scalar
};

enum class OrientationSource
{
    None = 0,
    IMU1 = 1,
    IMU2 = 2,
    VirtualIMU = 3,
    RigidBodyVIMU = 4
};

enum class OrientationFilter
{
    Accelerometer = 0,
    Complementary = 1,
    Madgwick = 2,
    Mahony = 3,
    MEKF = 4
};

struct DataStream
{
    std::string name;
    StreamType type;
};

struct IMUStream : public DataStream
{
    IMUSample sample;
};

struct QuaternionStream : public DataStream
{
    Quaternion quaternion;
};

struct IMUMount
{
    Quaternion rotation;
    Vec3 position;
};

class IMUPipeline
{
public:
    IMUPipeline();

    //--------------------------------------------------
    // Main update
    //--------------------------------------------------

    void update(const RawIMUPacket& raw);

    void setOrientationSource(OrientationSource source);
    void setOrientationFilter(OrientationFilter filter);

    void setMount1Rotation(float x, float y, float z);
    void setMount2Rotation(float x, float y, float z);

    const Quaternion& orientation() const;
    float deltaTime() const;
    //--------------------------------------------------
    // Input packet
    //--------------------------------------------------

    const RawIMUPacket& rawPacket() const;

    //--------------------------------------------------
    // Stream outputs
    //--------------------------------------------------

    const IMUStream& imu1() const;
    const IMUStream& imu2() const;
    const IMUStream& bodyIMU1() const;
    const IMUStream& bodyIMU2() const;
    const IMUStream& virtualIMU() const;
    const IMUStream& rigidBodyIMU() const;

    const QuaternionStream& madgwick() const;
    const QuaternionStream& mahony() const;
    const QuaternionStream& ekf() const;

    //--------------------------------------------------
    // Stream Registry
    //--------------------------------------------------

    const std::vector<DataStream*>& streams() const;

private:

    //--------------------------------------------------
    // Input
    //--------------------------------------------------

    RawIMUPacket m_raw;

    //--------------------------------------------------
    // Streams
    //--------------------------------------------------

    IMUStream m_imu1;
    IMUStream m_imu2;

    IMUStream m_bodyIMU1;
    IMUStream m_bodyIMU2;

    IMUMount m_mount1;
    IMUMount m_mount2;

    IMUStream m_virtualIMU;
    IMUStream m_rigidBodyIMU;

    Vec3 m_lastOmega;
    bool m_hasLastOmega = false;

    //--------------------------------
    // Selected pipeline state
    //--------------------------------

    OrientationSource m_orientationSource = OrientationSource::None;

    OrientationFilter m_orientationFilter = OrientationFilter::Accelerometer;

    const IMUStream* m_selectedIMU = nullptr;

    Quaternion m_orientation;

    AccelerometerFilter m_accelerometerFilter;
    ComplementaryFilter m_complementaryFilter;
    MadgwickFilter m_madgwickFilter;
    MahonyFilter m_mahonyFilter;
    MEKFFilter m_ekfFilter;

    //--------------------------------
    // Quaternion outputs
    //--------------------------------

    uint32_t m_lastTimestamp = 0;
    bool m_hasLastTimestamp = false;
    float m_deltaTime = 1.0f / 60.0f;

    QuaternionStream m_madgwick;
    QuaternionStream m_mahony;
    QuaternionStream m_ekf;

    Vec3 m_lastRigidOmega;
    bool m_hasLastRigidOmega = false;

    //--------------------------------------------------
    // Registry
    //--------------------------------------------------

    std::vector<DataStream*> m_streams;

};
#include <cmath>

#include "imu_pipeline.h"

static float degreesToRadians(float degrees)
{
    return degrees * 3.14159265f / 180.0f;
}

static Quaternion quaternionFromEulerDegrees(
    float xDeg,
    float yDeg,
    float zDeg
)
{
    float x = degreesToRadians(xDeg);
    float y = degreesToRadians(yDeg);
    float z = degreesToRadians(zDeg);

    float cx = std::cos(x * 0.5f);
    float sx = std::sin(x * 0.5f);

    float cy = std::cos(y * 0.5f);
    float sy = std::sin(y * 0.5f);

    float cz = std::cos(z * 0.5f);
    float sz = std::sin(z * 0.5f);

    Quaternion qx;
    qx.w = cx;
    qx.x = sx;

    Quaternion qy;
    qy.w = cy;
    qy.y = sy;

    Quaternion qz;
    qz.w = cz;
    qz.z = sz;

    // Rotation order: X, then Y, then Z
    Quaternion q = qz * (qy * qx);

    q.normalize();

    return q;
}

static Vec3 rotateVector(
    const Quaternion& q,
    const Vec3& v
)
{
    Quaternion p;
    p.w = 0.0f;
    p.x = v.x;
    p.y = v.y;
    p.z = v.z;

    Quaternion qInv;
    qInv.w =  q.w;
    qInv.x = -q.x;
    qInv.y = -q.y;
    qInv.z = -q.z;

    Quaternion result = (q * p) * qInv;

    return Vec3(
        result.x,
        result.y,
        result.z
    );
}

static IMUSample applyMount(
    const IMUSample& raw,
    const IMUMount& mount
)
{
    IMUSample out;

    Vec3 accel =
        rotateVector(
            mount.rotation,
            Vec3(raw.ax, raw.ay, raw.az)
        );

    Vec3 gyro =
        rotateVector(
            mount.rotation,
            Vec3(raw.gx, raw.gy, raw.gz)
        );

    Vec3 mag =
        rotateVector(
            mount.rotation,
            Vec3(raw.mx, raw.my, raw.mz)
        );

    out.ax = accel.x;
    out.ay = accel.y;
    out.az = accel.z;

    out.gx = gyro.x;
    out.gy = gyro.y;
    out.gz = gyro.z;

    out.mx = mag.x;
    out.my = mag.y;
    out.mz = mag.z;

    return out;
}

static Vec3 accelVector(const IMUSample& s)
{
    return Vec3(s.ax, s.ay, s.az);
}

static Vec3 gyroVectorRad(const IMUSample& s)
{
    const float degToRad = 3.14159265f / 180.0f;

    return Vec3(
        s.gx * degToRad,
        s.gy * degToRad,
        s.gz * degToRad
    );
}

static Vec3 magVector(const IMUSample& s)
{
    return Vec3(s.mx, s.my, s.mz);
}

static void writeSample(
    IMUSample& out,
    const Vec3& accel,
    const Vec3& gyroRad,
    const Vec3& mag)
{
    const float radToDeg = 180.0f / 3.14159265f;

    out.ax = accel.x;
    out.ay = accel.y;
    out.az = accel.z;

    out.gx = gyroRad.x * radToDeg;
    out.gy = gyroRad.y * radToDeg;
    out.gz = gyroRad.z * radToDeg;

    out.mx = mag.x;
    out.my = mag.y;
    out.mz = mag.z;
}

IMUPipeline::IMUPipeline()
{
    //-------------------------------
    // IMU Streams
    //-------------------------------

    m_imu1.name = "IMU 1";
    m_imu1.type = StreamType::IMU;

    m_imu2.name = "IMU 2";
    m_imu2.type = StreamType::IMU;

    m_bodyIMU1.name = "Body IMU 1";
    m_bodyIMU1.type = StreamType::IMU;

    m_bodyIMU2.name = "Body IMU 2";
    m_bodyIMU2.type = StreamType::IMU;

    m_virtualIMU.name = "Virtual IMU";
    m_virtualIMU.type = StreamType::IMU;

    m_rigidBodyIMU.name = "Rigid Body VIMU";
    m_rigidBodyIMU.type = StreamType::IMU;
    //--------------------------------
    // Mount orientations
    //--------------------------------


    //--------------------------------
    // Mount positions rigid-body VIMU
    //--------------------------------

    // IMU 1
    m_mount1.rotation =
        quaternionFromEulerDegrees(
            0.0f,
            0.0f,
            0.0f
        );
    // IMU 2
    // 180° flipped about Z
    m_mount2.rotation =
        quaternionFromEulerDegrees(
            0.0f,
            0.0f,
            180.0f
        );
    m_mount1.position = Vec3(
        0.0155f,
        0.0255f,
        0.0f
    );
    m_mount2.position = Vec3(
       -0.0115f,
        0.0115f,
        0.0f
    );

    //-------------------------------
    // Quaternion Streams
    //-------------------------------

    m_madgwick.name = "Madgwick";
    m_madgwick.type = StreamType::Quaternion;

    m_mahony.name = "Mahony";
    m_mahony.type = StreamType::Quaternion;

    m_ekf.name = "EKF";
    m_ekf.type = StreamType::Quaternion;

    //--------------------------------
    // Registry
    //--------------------------------

    m_streams =
    {
        &m_imu1,
        &m_imu2,

        &m_bodyIMU1,
        &m_bodyIMU2,

        &m_virtualIMU,
        &m_rigidBodyIMU,

        &m_madgwick,
        &m_mahony,
        &m_ekf
    };

}

void IMUPipeline::update(const RawIMUPacket& raw)
{
    //--------------------------------
    // Input
    //--------------------------------

    m_raw = raw;

    //--------------------------------
    // Raw streams
    //--------------------------------

    m_imu1.sample = raw.imu1;
    m_imu2.sample = raw.imu2;

    //--------------------------------------------------
    // Delta Time
    //--------------------------------------------------

    float dt = 1.0f / 60.0f;

    if (m_hasLastTimestamp)
    {
        uint32_t delta =
            raw.timestamp - m_lastTimestamp;

        dt = static_cast<float>(delta) / 1000.0f;

        if (dt <= 0.0f || dt > 1.0f)
            dt = 1.0f / 60.0f;
    }

    m_lastTimestamp = raw.timestamp;
    m_hasLastTimestamp = true;


    //--------------------------------
    // Body frame
    //--------------------------------

    m_bodyIMU1.sample =
        applyMount(
            m_imu1.sample,
            m_mount1
        );

    m_bodyIMU2.sample =
        applyMount(
            m_imu2.sample,
            m_mount2
        );

    //--------------------------------
    // Average VIMU
    //--------------------------------

    m_virtualIMU.sample.ax =
        0.5f * (m_bodyIMU1.sample.ax + m_bodyIMU2.sample.ax);

    m_virtualIMU.sample.ay =
        0.5f * (m_bodyIMU1.sample.ay + m_bodyIMU2.sample.ay);

    m_virtualIMU.sample.az =
        0.5f * (m_bodyIMU1.sample.az + m_bodyIMU2.sample.az);

    m_virtualIMU.sample.gx =
        0.5f * (m_bodyIMU1.sample.gx + m_bodyIMU2.sample.gx);

    m_virtualIMU.sample.gy =
        0.5f * (m_bodyIMU1.sample.gy + m_bodyIMU2.sample.gy);

    m_virtualIMU.sample.gz =
        0.5f * (m_bodyIMU1.sample.gz + m_bodyIMU2.sample.gz);

    m_virtualIMU.sample.mx =
        0.5f * (m_bodyIMU1.sample.mx + m_bodyIMU2.sample.mx);

    m_virtualIMU.sample.my =
        0.5f * (m_bodyIMU1.sample.my + m_bodyIMU2.sample.my);

    m_virtualIMU.sample.mz =
        0.5f * (m_bodyIMU1.sample.mz + m_bodyIMU2.sample.mz);

    //--------------------------------
    // Rigid-Body VIMU
    //--------------------------------

    IMUMount rigidMount1;
    IMUMount rigidMount2;

    // Hardcoded physical sensor orientations
    rigidMount1.rotation =
        quaternionFromEulerDegrees(
            0.0f,
            0.0f,
            0.0f
        );

    rigidMount2.rotation =
        quaternionFromEulerDegrees(
            0.0f,
            0.0f,
            180.0f
        );

    // Hardcoded sensor positions in body frame, meters
    rigidMount1.position =
        Vec3(
            0.0155f,
            0.0255f,
            0.0f
        );

    rigidMount2.position =
        Vec3(
        -0.0115f,
            0.0115f,
            0.0f
        );

    IMUSample rigidBody1 =
        applyMount(
            m_imu1.sample,
            rigidMount1
        );

    IMUSample rigidBody2 =
        applyMount(
            m_imu2.sample,
            rigidMount2
        );

    Vec3 a1 = accelVector(rigidBody1);
    Vec3 a2 = accelVector(rigidBody2);

    Vec3 w1 = gyroVectorRad(rigidBody1);
    Vec3 w2 = gyroVectorRad(rigidBody2);

    Vec3 m1 = magVector(rigidBody1);
    Vec3 m2 = magVector(rigidBody2);

    Vec3 omega =
        (w1 + w2) * 0.5f;

    Vec3 alpha;

    if (m_hasLastRigidOmega && dt > 0.0f)
    {
        alpha =
            (omega - m_lastRigidOmega) / dt;
    }
    else
    {
        alpha =
            Vec3(
                0.0f,
                0.0f,
                0.0f
            );
    }

    m_lastRigidOmega = omega;
    m_hasLastRigidOmega = true;

    Vec3 a1Origin =
        a1
        - Vec3::cross(alpha, rigidMount1.position)
        - Vec3::cross(
            omega,
            Vec3::cross(
                omega,
                rigidMount1.position
            )
        );

    Vec3 a2Origin =
        a2
        - Vec3::cross(alpha, rigidMount2.position)
        - Vec3::cross(
            omega,
            Vec3::cross(
                omega,
                rigidMount2.position
            )
        );

    Vec3 accel =
        (a1Origin + a2Origin) * 0.5f;

    Vec3 mag =
        (m1 + m2) * 0.5f;

    writeSample(
        m_rigidBodyIMU.sample,
        accel,
        omega,
        mag
    );

    //--------------------------------
    // Selected IMU Source
    //--------------------------------

    switch (m_orientationSource)
    {
        case OrientationSource::None:
            m_selectedIMU = nullptr;
            break;

        case OrientationSource::IMU1:
            m_selectedIMU = &m_bodyIMU1;
            break;

        case OrientationSource::IMU2:
            m_selectedIMU = &m_bodyIMU2;
            break;

        case OrientationSource::VirtualIMU:
            m_selectedIMU = &m_virtualIMU;
            break;

        case OrientationSource::RigidBodyVIMU:
            m_selectedIMU = &m_rigidBodyIMU;
            break;
    }

   
    //--------------------------------
    // Selected output orientation
    //--------------------------------

    if (!m_selectedIMU)
    {
        m_orientation = Quaternion();
    }
    else
    {
        switch (m_orientationFilter)
        {
            case OrientationFilter::Accelerometer:

                m_accelerometerFilter.update(m_selectedIMU->sample,0.0f);
                m_orientation = m_accelerometerFilter.orientation();
                break;

            case OrientationFilter::Complementary:

                m_complementaryFilter.update(m_selectedIMU->sample, dt);
                m_orientation = m_complementaryFilter.orientation();
                break;
            
            case OrientationFilter::Madgwick:
                m_madgwickFilter.update(
                    m_selectedIMU->sample,dt);

                m_orientation =
                    m_madgwickFilter.orientation();

                break;
            case OrientationFilter::Mahony:

                m_mahonyFilter.update(
                    m_selectedIMU->sample,
                    dt
                );

                m_orientation =
                    m_mahonyFilter.orientation();

                break;

            case OrientationFilter::MEKF:

                m_ekfFilter.update(
                    m_selectedIMU->sample,
                    dt
                );

                m_orientation =
                    m_ekfFilter.orientation();

                break;

        }
    }

    m_orientation.normalize();

}

const RawIMUPacket& IMUPipeline::rawPacket() const
{
    return m_raw;
}

const IMUStream& IMUPipeline::imu1() const
{
    return m_imu1;
}

const IMUStream& IMUPipeline::imu2() const
{
    return m_imu2;
}

const IMUStream& IMUPipeline::bodyIMU1() const
{
    return m_bodyIMU1;
}

const IMUStream& IMUPipeline::bodyIMU2() const
{
    return m_bodyIMU2;
}

const IMUStream& IMUPipeline::virtualIMU() const
{
    return m_virtualIMU;
}

const QuaternionStream& IMUPipeline::madgwick() const
{
    return m_madgwick;
}

const QuaternionStream& IMUPipeline::mahony() const
{
    return m_mahony;
}

const QuaternionStream& IMUPipeline::ekf() const
{
    return m_ekf;
}

const std::vector<DataStream*>&
IMUPipeline::streams() const
{
    return m_streams;
}

void IMUPipeline::setOrientationSource(
    OrientationSource source
)
{
    m_orientationSource = source;
}

void IMUPipeline::setOrientationFilter(
    OrientationFilter filter
)
{
    m_orientationFilter = filter;
}

const Quaternion& IMUPipeline::orientation() const
{
    return m_orientation;
}

void IMUPipeline::setMount1Rotation(float x, float y, float z)
{
    m_mount1.rotation =
        quaternionFromEulerDegrees(
            x,
            y,
            z
        );
}

void IMUPipeline::setMount2Rotation(float x, float y, float z)
{
    m_mount2.rotation =
        quaternionFromEulerDegrees(
            x,
            y,
            z
        );
}
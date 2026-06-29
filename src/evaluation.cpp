#include "evaluation.h"
#include "quaternion.h"
#include "imu_pipeline.h"

#include <fstream>
#include <iomanip>
#include <cmath>

void EvaluationRecorder::clear()
{
    m_frames.clear();
}

void EvaluationRecorder::push(const EvaluationFrame& frame)
{
    m_frames.push_back(frame);
}

bool EvaluationRecorder::saveCSV(const std::string& filename)
{
    std::ofstream file(filename);

    if (!file.is_open())
        return false;

    file << std::fixed << std::setprecision(6);

    file

    << "Time,"

    << "AccelRoll,AccelPitch,AccelYaw,"

    << "CompRoll,CompPitch,CompYaw,"

    << "MahonyRoll,MahonyPitch,MahonyYaw,"

    << "MadgwickRoll,MadgwickPitch,MadgwickYaw,"

    << "MEKFRoll,MEKFPitch,MEKFYaw\n";

    for(const auto& f : m_frames)
    {
        file

        << f.time << ','

        << f.accelRoll << ','
        << f.accelPitch << ','
        << f.accelYaw << ','

        << f.compRoll << ','
        << f.compPitch << ','
        << f.compYaw << ','

        << f.mahonyRoll << ','
        << f.mahonyPitch << ','
        << f.mahonyYaw << ','

        << f.madgwickRoll << ','
        << f.madgwickPitch << ','
        << f.madgwickYaw << ','

        << f.mekfRoll << ','
        << f.mekfPitch << ','
        << f.mekfYaw

        << '\n';
    }

    return true;
}

static void QuaternionToEuler(
    const Quaternion& q,
    float& roll,
    float& pitch,
    float& yaw)
{
    float sinr =
        2.0f * (q.w * q.x + q.y * q.z);

    float cosr =
        1.0f - 2.0f * (q.x * q.x + q.y * q.y);

    roll = std::atan2(sinr, cosr);

    float sinp =
        2.0f * (q.w * q.y - q.z * q.x);

    if (std::fabs(sinp) >= 1.0f)
        pitch = std::copysign(3.14159265f / 2.0f, sinp);
    else
        pitch = std::asin(sinp);

    float siny =
        2.0f * (q.w * q.z + q.x * q.y);

    float cosy =
        1.0f - 2.0f * (q.y * q.y + q.z * q.z);

    yaw = std::atan2(siny, cosy);

    constexpr float RAD2DEG = 57.2957795f;

    roll *= RAD2DEG;
    pitch *= RAD2DEG;
    yaw *= RAD2DEG;
}

void EvaluationManager::quaternionToEuler(
    const Quaternion& q,
    float& roll,
    float& pitch,
    float& yaw)
{
    QuaternionToEuler(
        q,
        roll,
        pitch,
        yaw
    );
}

void EvaluationManager::update(
    const IMUPipeline& pipeline,
    float dt)
{
    m_time += dt;

    recordSource(
        m_imu1,
        pipeline.bodyIMU1().sample,
        dt,

        m_accelIMU1,
        m_compIMU1,
        m_mahonyIMU1,
        m_madgwickIMU1,
        m_mekfIMU1,

        m_time
    );

    recordSource(
        m_imu2,
        pipeline.bodyIMU2().sample,
        dt,

        m_accelIMU2,
        m_compIMU2,
        m_mahonyIMU2,
        m_madgwickIMU2,
        m_mekfIMU2,

        m_time
    );

    recordSource(
        m_average,
        pipeline.virtualIMU().sample,
        dt,

        m_accelAverage,
        m_compAverage,
        m_mahonyAverage,
        m_madgwickAverage,
        m_mekfAverage,

        m_time
    );

    recordSource(
        m_rigid,
        pipeline.rigidBodyIMU().sample,
        dt,

        m_accelRigid,
        m_compRigid,
        m_mahonyRigid,
        m_madgwickRigid,
        m_mekfRigid,

        m_time
    );
}

void EvaluationManager::recordSource(
    EvaluationRecorder& recorder,
    const IMUSample& sample,
    float dt,

    AccelerometerFilter& accel,
    ComplementaryFilter& comp,
    MahonyFilter& mahony,
    MadgwickFilter& madgwick,
    MEKFFilter& mekf,

    double time
)
{
    accel.update(sample, dt);
    comp.update(sample, dt);
    mahony.update(sample, dt);
    madgwick.update(sample, dt);
    mekf.update(sample, dt);

    EvaluationFrame frame;

    frame.time = time;

    quaternionToEuler(
        accel.orientation(),
        frame.accelRoll,
        frame.accelPitch,
        frame.accelYaw
    );

    quaternionToEuler(
        comp.orientation(),
        frame.compRoll,
        frame.compPitch,
        frame.compYaw
    );

    quaternionToEuler(
        mahony.orientation(),
        frame.mahonyRoll,
        frame.mahonyPitch,
        frame.mahonyYaw
    );

    quaternionToEuler(
        madgwick.orientation(),
        frame.madgwickRoll,
        frame.madgwickPitch,
        frame.madgwickYaw
    );

    quaternionToEuler(
        mekf.orientation(),
        frame.mekfRoll,
        frame.mekfPitch,
        frame.mekfYaw
    );

    recorder.push(frame);
}

void EvaluationManager::saveAll()
{
    m_imu1.saveCSV(
        "IMU1.csv"
    );

    m_imu2.saveCSV(
        "IMU2.csv"
    );

    m_average.saveCSV(
        "AverageVIMU.csv"
    );

    m_rigid.saveCSV(
        "RigidBodyVIMU.csv"
    );
}
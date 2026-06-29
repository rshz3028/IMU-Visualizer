#pragma once

#include <string>
#include <vector>

#include "filter/accelerometer_filter.h"
#include "filter/complementary_filter.h"
#include "filter/mahony_filter.h"
#include "filter/madgwick_filter.h"
#include "filter/mekf_filter.h"


class IMUPipeline;

struct EvaluationFrame
{
    double time;

    float accelRoll;
    float accelPitch;
    float accelYaw;

    float compRoll;
    float compPitch;
    float compYaw;

    float mahonyRoll;
    float mahonyPitch;
    float mahonyYaw;

    float madgwickRoll;
    float madgwickPitch;
    float madgwickYaw;

    float mekfRoll;
    float mekfPitch;
    float mekfYaw;
};

class EvaluationRecorder
{
public:

    void clear();

    void push(const EvaluationFrame& frame);

    bool saveCSV(const std::string& filename);

private:

    std::vector<EvaluationFrame> m_frames;
};

class EvaluationManager
{
public:

    void update(
        const IMUPipeline& pipeline,
        float dt
    );

    void saveAll();

private:

    EvaluationRecorder m_imu1;
    EvaluationRecorder m_imu2;
    EvaluationRecorder m_average;
    EvaluationRecorder m_rigid;

    // IMU1
    AccelerometerFilter m_accelIMU1;
    ComplementaryFilter m_compIMU1;
    MahonyFilter         m_mahonyIMU1;
    MadgwickFilter       m_madgwickIMU1;
    MEKFFilter           m_mekfIMU1;

    // IMU2
    AccelerometerFilter m_accelIMU2;
    ComplementaryFilter m_compIMU2;
    MahonyFilter         m_mahonyIMU2;
    MadgwickFilter       m_madgwickIMU2;
    MEKFFilter           m_mekfIMU2;

    // Average
    AccelerometerFilter m_accelAverage;
    ComplementaryFilter m_compAverage;
    MahonyFilter         m_mahonyAverage;
    MadgwickFilter       m_madgwickAverage;
    MEKFFilter           m_mekfAverage;

    // Rigid Body
    AccelerometerFilter m_accelRigid;
    ComplementaryFilter m_compRigid;
    MahonyFilter         m_mahonyRigid;
    MadgwickFilter       m_madgwickRigid;
    MEKFFilter           m_mekfRigid;

    double m_time = 0.0;

    void recordSource(
        EvaluationRecorder& recorder,
        const IMUSample& sample,
        float dt,

        AccelerometerFilter& accel,
        ComplementaryFilter& comp,
        MahonyFilter& mahony,
        MadgwickFilter& madgwick,
        MEKFFilter& mekf,

        double time
    );

    static void quaternionToEuler(
        const Quaternion& q,
        float& roll,
        float& pitch,
        float& yaw
    );

};
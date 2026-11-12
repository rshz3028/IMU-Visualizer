#include "scene.h"
#include "primitive_type.h"
#include "transform.h"
#include "quaternion.h"
#include "imu_pipeline.h"
#include "evaluation.h"

#pragma once

struct SDL_Window;

class Editor
{
public:

    bool initialize(SDL_Window* window,void* glContext);

    void beginFrame();
    void draw(
    Scene& scene,
    EvaluationManager& evaluation,
    SerialIMU& serial,
    const char* rawLogPath
);

    void endFrame();
    void shutdown();

    bool consumeSpawnRequest(PrimitiveType& type);
    OrientationSource orientationSource() const;
    OrientationFilter orientationFilter() const;
    OrientationSource compareSource() const;
    OrientationFilter compareFilter() const;

    const float* imu1Rotation() const;
    const float* imu2Rotation() const;

    private:

    PrimitiveType m_selectedPrimitive = PrimitiveType::Cube;
    bool m_spawnRequested = false;

    int m_orientationSource = 0;
    int m_orientationFilter = 0;

    int m_compareSource = 0;
    int m_compareFilter = 0;

    float m_imu1Rotation[3] = { 0.0f, 0.0f, 0.0f };
    float m_imu2Rotation[3] = { 0.0f, 0.0f, 0.0f };

};
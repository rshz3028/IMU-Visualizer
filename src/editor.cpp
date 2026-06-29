#include "editor.h"

#include <imgui.h>

#include <backends/imgui_impl_sdl3.h>
#include <backends/imgui_impl_opengl3.h>

#include <SDL3/SDL.h>


bool Editor::initialize(
    SDL_Window* window,
    void* glContext)
{
    IMGUI_CHECKVERSION();

    ImGui::CreateContext();

    ImGui::StyleColorsDark();

    ImGui_ImplSDL3_InitForOpenGL(
        window,
        glContext
    );

    ImGui_ImplOpenGL3_Init(
        "#version 330"
    );

    return true;
}

void Editor::beginFrame()
{
    ImGui_ImplOpenGL3_NewFrame();

    ImGui_ImplSDL3_NewFrame();

    ImGui::NewFrame();
}

void Editor::draw(Scene& scene)
{
    ImGui::Begin("IMU_Test_Visualizer");

    static int current = 0;

    const char* primitives[] =
    {
        "Cube",
        "Cylinder"
    };

    ImGui::Combo(
        "Primitive",
        &current,
        primitives,
        IM_ARRAYSIZE(primitives)
    );

    m_selectedPrimitive =
        (current == 0)
        ? PrimitiveType::Cube
        : PrimitiveType::Cylinder;

    if (ImGui::Button("Spawn"))
    {
        m_spawnRequested = true;
        
    }
    ImGui::Separator();

    ImGui::Text("Pipeline");

    const char* orientationSources[] =
    {
        "None",
        "IMU 1",
        "IMU 2",
        "Average VIMU",
        "Rigid Body VIMU"
    };

    ImGui::Combo(
        "Orientation Source",
        &m_orientationSource,
        orientationSources,
        IM_ARRAYSIZE(orientationSources)
    );

    const char* orientationFilters[] =
    {
        "Accelerometer",
        "Complementary",
        "Madgwick",
        "Mahony",
        "MEKF"
    };

    ImGui::Combo(
        "Orientation Filter",
        &m_orientationFilter,
        orientationFilters,
        IM_ARRAYSIZE(orientationFilters)
    );

    ImGui::Separator();
    //--------------------------------
    // IMU Mounts
    //--------------------------------

    ImGui::Text("IMU 1 Mount");

    ImGui::DragFloat(
        "IMU1 X Rotation",
        &m_imu1Rotation[0],
        1.0f,
        -180.0f,
        180.0f,
        "%.1f deg"
    );

    ImGui::DragFloat(
        "IMU1 Y Rotation",
        &m_imu1Rotation[1],
        1.0f,
        -180.0f,
        180.0f,
        "%.1f deg"
    );

    ImGui::DragFloat(
        "IMU1 Z Rotation",
        &m_imu1Rotation[2],
        1.0f,
        -180.0f,
        180.0f,
        "%.1f deg"
    );

    ImGui::Separator();

    ImGui::Text("IMU 2 Mount");

    ImGui::DragFloat(
        "IMU2 X Rotation",
        &m_imu2Rotation[0],
        1.0f,
        -180.0f,
        180.0f,
        "%.1f deg"
    );

    ImGui::DragFloat(
        "IMU2 Y Rotation",
        &m_imu2Rotation[1],
        1.0f,
        -180.0f,
        180.0f,
        "%.1f deg"
    );

    ImGui::DragFloat(
        "IMU2 Z Rotation",
        &m_imu2Rotation[2],
        1.0f,
        -180.0f,
        180.0f,
        "%.1f deg"
    );

    ImGui::Separator();

    if (SceneObject* object = scene.imuObject())
    {
        ImGui::Text("Transform");

        ImGui::DragFloat3(
            "Position",
            &object->transform.position.x,
            0.01f
        );

        ImGui::DragFloat3(
            "Scale",
            &object->transform.scale.x,
            0.01f
        );

        ImGui::Separator();

        const Quaternion& q =
            object->transform.rotation;

        ImGui::Text("Sensor Orientation");

        ImGui::Text("W : %.6f", q.w);
        ImGui::Text("X : %.6f", q.x);
        ImGui::Text("Y : %.6f", q.y);
        ImGui::Text("Z : %.6f", q.z);

        float magnitude =
            sqrtf(
                q.w*q.w +
                q.x*q.x +
                q.y*q.y +
                q.z*q.z
            );

        ImGui::Text(
            "Quaternion Norm : %.6f",
            magnitude
        );
    }

    ImGui::End();
}

bool Editor::consumeSpawnRequest(
    PrimitiveType& type
)
{
    if (!m_spawnRequested)
        return false;

    type = m_selectedPrimitive;

    m_spawnRequested = false;

    return true;
}

OrientationSource Editor::orientationSource() const
{
    return static_cast<OrientationSource>(
        m_orientationSource
    );
}

OrientationFilter Editor::orientationFilter() const
{
    return static_cast<OrientationFilter>(
        m_orientationFilter
    );
}

const float* Editor::imu1Rotation() const
{
    return m_imu1Rotation;
}

const float* Editor::imu2Rotation() const
{
    return m_imu2Rotation;
}

void Editor::endFrame()
{
    ImGui::Render();

    ImGui_ImplOpenGL3_RenderDrawData(
        ImGui::GetDrawData()
    );
}

void Editor::shutdown()
{
    ImGui_ImplOpenGL3_Shutdown();

    ImGui_ImplSDL3_Shutdown();

    ImGui::DestroyContext();
}
#include <iostream>

#include "renderer.h"
#include "input.h"
#include "mesh.h"
#include "shader.h"
#include "camera.h"
#include "utils.h"
#include "tools_geometry.h"
#include "editor.h"
#include "scene.h"
#include "serial_imu.h"
#include "imu_pipeline.h"
#include "evaluation.h"

//--------------------------------------------------
// Serial configuration
//--------------------------------------------------

static const char* IMU_SERIAL_PORT = "COM12";
static const int   IMU_BAUD_RATE   = 921600;

// Log every raw packet to raw_log.csv from startup
// (can also be toggled in the editor).
static const bool  RAW_LOG_ENABLED = true;
static const char* RAW_LOG_PATH    = "raw_log.csv";

int main(int argc, char** argv)
{
    // Optional: --replay <raw_log.csv> [--speed <x>]
    std::string replayPath;
    float replaySpeed = 1.0f;

    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];

        if (arg == "--replay" && i + 1 < argc)
            replayPath = argv[++i];
        else if (arg == "--speed" && i + 1 < argc)
            replaySpeed = std::strtof(argv[++i], nullptr);
    }

    Renderer renderer;
    Input input;
    Camera camera;
    Editor editor;
    Scene scene;
    SerialIMU imu;
    IMUPipeline pipeline;
    EvaluationManager evaluation;

    if (!renderer.initialize())
    {
        std::cerr << "Failed to initialize renderer.\n";
        return -1;
    }

    input.initialize(
        renderer.getWindow()
    );

    editor.initialize(
        renderer.getWindow(),
        renderer.getContext()
    );

    if (!replayPath.empty())
    {
        if (!imu.openReplay(replayPath, replaySpeed))
            std::cerr << "Failed to open replay file: " << replayPath << "\n";
    }
    else if (!imu.open(IMU_SERIAL_PORT, IMU_BAUD_RATE))
    {
        std::cerr << "Failed to open IMU serial port.\n";
    }

    // Don't re-log a recording that's being replayed.
    if (RAW_LOG_ENABLED && replayPath.empty())
    {
        imu.startRawLog(RAW_LOG_PATH);
    }

    Shader basicShader;

    if (!basicShader.loadFromFiles(
        Utils::assetPath("shaders/basic.vert").string(),
        Utils::assetPath("shaders/basic.frag").string()
    ))
    {
        std::cerr << "Failed to load shader.\n";
        return -1;
    }

    // -------------------------
    // Grid
    // -------------------------

    SceneObject grid;

    grid.mesh.upload(
        ToolsGeometry::createGrid(
            20,
            0.1f
        )
    );

    grid.primitive = GL_LINES;

    scene.addObject(
        std::move(grid)
    );

    // -------------------------
    // Axes
    // -------------------------

    SceneObject axes;

    axes.mesh.upload(
        ToolsGeometry::createAxes(
            1.0f
        )
    );

    axes.primitive = GL_LINES;

    scene.addObject(
        std::move(axes)
    );

    // -------------------------

    bool running = true;

    while (running)
    {
        input.update(running);

        camera.update(
            input,
            1.0f / 60.0f
        );
        pipeline.setOrientationSource(editor.orientationSource());
        pipeline.setOrientationFilter(editor.orientationFilter());

        const float* imu1 = editor.imu1Rotation();
        const float* imu2 = editor.imu2Rotation();

        pipeline.setMount1Rotation(imu1[0],imu1[1],imu1[2]);
        pipeline.setMount2Rotation(imu2[0],imu2[1],imu2[2]);

        // Read all pending bytes, then process every queued
        // packet (not just the newest one) this frame.
        imu.update();

        RawIMUPacket packet;
        bool receivedPacket = false;

        while (imu.nextPacket(packet))
        {
            pipeline.update(packet);
            evaluation.update(pipeline,pipeline.deltaTime());
            receivedPacket = true;
        }

        if (receivedPacket)
        {
            if (SceneObject* object = scene.imuObject())
            {
                object->transform.rotation =
                    pipeline.orientation();
            }
        }

        renderer.beginFrame();

        for (auto& object : scene.objects())
        {
            renderer.draw(
                object.mesh,
                basicShader,
                camera,
                object.transform,
                object.primitive
            );
        }

        editor.beginFrame();
        editor.draw(
            scene,
            evaluation,
            imu,
            RAW_LOG_PATH
        );

        PrimitiveType type;
        if (editor.consumeSpawnRequest(type))
        {
            scene.spawnIMUObject(type);
        }

        editor.endFrame();

        renderer.endFrame();
    }

    evaluation.saveAll();
    imu.close();
    basicShader.destroy();
    renderer.shutdown();

    return 0;
}
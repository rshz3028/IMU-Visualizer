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

int main()
{
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

    if (!imu.open("COM12", 115200))
    {
        std::cerr << "Failed to open IMU serial port.\n";
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

        if (imu.update())
        {
            pipeline.update(imu.packet());
            evaluation.update(pipeline,pipeline.deltaTime());
            if (SceneObject* object = scene.imuObject())
            {
                object->transform.rotation =
                    pipeline.orientation();
            }
            const RawIMUPacket& packet =
                pipeline.rawPacket();   
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
        editor.draw(scene);

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
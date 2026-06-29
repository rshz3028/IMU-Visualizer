#include <algorithm>
#include <utility>

#include "scene.h"
#include "tools_geometry.h"


void Scene::addObject(SceneObject&& object)
{
    m_objects.push_back(std::move(object));
}

void Scene::clear()
{
    m_objects.clear();
}

void Scene::spawnIMUObject(
    PrimitiveType type
)
{
    if (m_imuObject)
    {
        auto it =
            std::find_if(
                m_objects.begin(),
                m_objects.end(),
                [&](const SceneObject& object)
                {
                    return &object == m_imuObject;
                });

        if (it != m_objects.end())
            m_objects.erase(it);

        m_imuObject = nullptr;
    }

    SceneObject object;

    switch (type)
    {
        case PrimitiveType::Cube:

            object.mesh.upload(
                ToolsGeometry::createCube(1.0f)
            );

            object.primitive = GL_TRIANGLES;

            break;

        case PrimitiveType::Cylinder:

            // later

            break;
    }

    object.transform.position =
    {
        0.0f,
        0.5f,
        0.0f
    };

    m_objects.push_back(
        std::move(object)
    );

    m_imuObject = &m_objects.back();
}

SceneObject* Scene::imuObject()
{
    return m_imuObject;
}

std::vector<SceneObject>& Scene::objects()
{
    return m_objects;
}

const std::vector<SceneObject>& Scene::objects() const
{
    return m_objects;
}
#pragma once

#include <vector>
#include <glad/gl.h>

#include "mesh.h"
#include "math3d.h"
#include "primitive_type.h"
#include "transform.h"

struct SceneObject
{
    Mesh mesh;
    Transform transform;
    GLenum primitive = GL_TRIANGLES;
};

class Scene
{
public:
    void addObject(SceneObject&& object);
    void clear();

    std::vector<SceneObject>& objects();
    const std::vector<SceneObject>& objects() const;

    void spawn(PrimitiveType type);
    SceneObject* imuObject();

    void spawnIMUObject(PrimitiveType type);

private:
    std::vector<SceneObject> m_objects;
    SceneObject* m_imuObject = nullptr;
};
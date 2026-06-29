#pragma once

#include <SDL3/SDL.h>
#include <glad/gl.h>

#include "mesh.h"
#include "shader.h"
#include "camera.h"
#include "math3d.h"
#include "transform.h"

class Renderer
{
public:
    Renderer();
    ~Renderer();

    float aspectRatio() const;
    SDL_GLContext getContext() const;
    bool initialize();

    void beginFrame();

    void draw(
        const Mesh& mesh,
        const Shader& shader,
        const Camera& camera,
        const Transform& transform,
        GLenum primitive = GL_TRIANGLES
    );

    void endFrame();

    void shutdown();

    SDL_Window* getWindow() const;

private:
    SDL_Window* m_window;
    SDL_GLContext m_context;
};
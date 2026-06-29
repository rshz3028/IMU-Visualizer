#include "renderer.h"

#include <iostream>

Renderer::Renderer()
    : m_window(nullptr),
      m_context(nullptr)
{
}

Renderer::~Renderer()
{
    shutdown();
}

bool Renderer::initialize()
{
    if (!SDL_Init(SDL_INIT_VIDEO))
    {
        std::cerr << "SDL_Init failed: "
                  << SDL_GetError() << '\n';
        return false;
    }

    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 4);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 6);
    SDL_GL_SetAttribute(
        SDL_GL_CONTEXT_PROFILE_MASK,
        SDL_GL_CONTEXT_PROFILE_CORE
    );

    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);

    m_window = SDL_CreateWindow(
        "IMU Research",
        1280,
        720,
        SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE
    );

    if (!m_window)
    {
        std::cerr << "Failed to create window.\n";
        return false;
    }

    m_context = SDL_GL_CreateContext(m_window);

    if (!m_context)
    {
        std::cerr << "Failed to create OpenGL context.\n";
        return false;
    }

    if (!gladLoadGL((GLADloadfunc)SDL_GL_GetProcAddress))
    {
        std::cerr << "Failed to initialize GLAD.\n";
        return false;
    }

    SDL_GL_SetSwapInterval(1);

    glViewport(0, 0, 1280, 720);

    glEnable(GL_DEPTH_TEST);

    return true;
}

void Renderer::beginFrame()
{
    int width;
    int height;

    SDL_GetWindowSize(
        m_window,
        &width,
        &height
    );

    glViewport(
        0,
        0,
        width,
        height
    );

    glClearColor(
        0.12f,
        0.12f,
        0.15f,
        1.0f
    );

    glClear(
        GL_COLOR_BUFFER_BIT |
        GL_DEPTH_BUFFER_BIT
    );
}

void Renderer::draw(
    const Mesh& mesh,
    const Shader& shader,
    const Camera& camera,
    const Transform& transform,
    GLenum primitive
)
{
    shader.use();

    shader.setMat4(
        "model",
        transform.matrix()
    );

    shader.setMat4(
        "view",
        camera.getViewMatrix()
    );

    shader.setMat4(
        "projection",
        camera.getProjectionMatrix(
            aspectRatio()
        )
    );

    mesh.draw(primitive);
}

void Renderer::endFrame()
{
    SDL_GL_SwapWindow(m_window);
}

void Renderer::shutdown()
{
    if (m_context)
    {
        SDL_GL_DestroyContext(m_context);
        m_context = nullptr;
    }

    if (m_window)
    {
        SDL_DestroyWindow(m_window);
        m_window = nullptr;
    }

    SDL_Quit();
}

SDL_Window* Renderer::getWindow() const
{
    return m_window;
}

float Renderer::aspectRatio() const
{
    int width;
    int height;

    SDL_GetWindowSize(
        m_window,
        &width,
        &height
    );

    if (height == 0)
        return 1.0f;

    return static_cast<float>(width) /
           static_cast<float>(height);
}

SDL_GLContext Renderer::getContext() const
{
    return m_context;
}
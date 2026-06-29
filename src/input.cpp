#include "input.h"

#include <backends/imgui_impl_sdl3.h>

Input::Input()
{
    m_keyboard = SDL_GetKeyboardState(nullptr);

    m_mouseButtons = 0;

    m_mouseX = 0;
    m_mouseY = 0;

    m_deltaX = 0;
    m_deltaY = 0;

    m_wheel = 0.0f;

    m_window = nullptr;
    m_cursorLocked = false;

}

void Input::update(bool& running)
{
    SDL_Event event;

    m_deltaX = 0;
    m_deltaY = 0;

    m_wheel = 0.0f;

    while (SDL_PollEvent(&event))
    {
        ImGui_ImplSDL3_ProcessEvent(&event);
        switch (event.type)
        {
        case SDL_EVENT_QUIT:
            running = false;
            break;

        case SDL_EVENT_MOUSE_MOTION:
            m_mouseX = event.motion.x;
            m_mouseY = event.motion.y;

            m_deltaX += event.motion.xrel;
            m_deltaY += event.motion.yrel;
            break;

        case SDL_EVENT_MOUSE_WHEEL:
            m_wheel += event.wheel.y;
            break;
        }
    }

    m_mouseButtons = SDL_GetMouseState(nullptr, nullptr);
}

bool Input::keyDown(SDL_Scancode key) const
{
    return m_keyboard[key];
}

bool Input::mouseButtonDown(Uint8 button) const
{
    return (m_mouseButtons & SDL_BUTTON_MASK(button)) != 0;
}

bool Input::leftMouseDown() const
{
    return mouseButtonDown(SDL_BUTTON_LEFT);
}

bool Input::middleMouseDown() const
{
    return mouseButtonDown(SDL_BUTTON_MIDDLE);
}

bool Input::rightMouseDown() const
{
    return mouseButtonDown(SDL_BUTTON_RIGHT);
}

int Input::mouseX() const
{
    return m_mouseX;
}

int Input::mouseY() const
{
    return m_mouseY;
}

int Input::mouseDeltaX() const
{
    return m_deltaX;
}

int Input::mouseDeltaY() const
{
    return m_deltaY;
}

float Input::mouseWheel() const
{
    return m_wheel;
}

void Input::initialize(SDL_Window* window)
{
    m_window = window;
}

void Input::lockCursor(bool lock)
{
    if (m_cursorLocked == lock)
        return;

    m_cursorLocked = lock;

    if (m_window)
    {
        SDL_SetWindowRelativeMouseMode(
            m_window,
            lock
        );
    }
}
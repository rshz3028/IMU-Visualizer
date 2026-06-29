#pragma once

#include <SDL3/SDL.h>

class Input
{
public:
    Input();

    void update(bool& running);

    bool keyDown(SDL_Scancode key) const;

    bool mouseButtonDown(Uint8 button) const;

    bool leftMouseDown() const;
    bool middleMouseDown() const;
    bool rightMouseDown() const;

    int mouseX() const;
    int mouseY() const;

    int mouseDeltaX() const;
    int mouseDeltaY() const;

    float mouseWheel() const;

    void initialize(SDL_Window* window);
    void lockCursor(bool lock);
    bool cursorLocked() const;

private:
    const bool* m_keyboard;

    Uint32 m_mouseButtons;

    int m_mouseX;
    int m_mouseY;

    int m_deltaX;
    int m_deltaY;

    float m_wheel;

    SDL_Window* m_window;
    bool m_cursorLocked;
};
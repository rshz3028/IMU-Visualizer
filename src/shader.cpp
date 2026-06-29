#include "shader.h"

#include <fstream>
#include <iostream>
#include <sstream>

Shader::Shader()
    : m_program(0)
{
}

Shader::~Shader()
{
    destroy();
}

bool Shader::loadFromFiles(
    const std::string& vertexPath,
    const std::string& fragmentPath
)
{
    destroy();

    std::string vertexSource;
    std::string fragmentSource;

    if (!readFile(vertexPath, vertexSource))
    {
        std::cerr << "Failed to read vertex shader: "
                  << vertexPath << '\n';
        return false;
    }

    if (!readFile(fragmentPath, fragmentSource))
    {
        std::cerr << "Failed to read fragment shader: "
                  << fragmentPath << '\n';
        return false;
    }

    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);

    if (!compileShader(vertexShader, vertexSource, vertexPath))
    {
        glDeleteShader(vertexShader);
        glDeleteShader(fragmentShader);
        return false;
    }

    if (!compileShader(fragmentShader, fragmentSource, fragmentPath))
    {
        glDeleteShader(vertexShader);
        glDeleteShader(fragmentShader);
        return false;
    }

    m_program = glCreateProgram();

    glAttachShader(m_program, vertexShader);
    glAttachShader(m_program, fragmentShader);

    glLinkProgram(m_program);

    GLint success = 0;
    glGetProgramiv(m_program, GL_LINK_STATUS, &success);

    if (!success)
    {
        char log[1024];
        glGetProgramInfoLog(m_program, 1024, nullptr, log);

        std::cerr << "Shader program link failed:\n"
                  << log << '\n';

        glDeleteShader(vertexShader);
        glDeleteShader(fragmentShader);

        destroy();

        return false;
    }

    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    return true;
}

void Shader::use() const
{
    glUseProgram(m_program);
}

void Shader::destroy()
{
    if (m_program)
    {
        glDeleteProgram(m_program);
        m_program = 0;
    }
}

GLuint Shader::id() const
{
    return m_program;
}

bool Shader::readFile(
    const std::string& path,
    std::string& output
)
{
    std::ifstream file(path);

    if (!file.is_open())
    {
        return false;
    }

    std::stringstream buffer;
    buffer << file.rdbuf();

    output = buffer.str();

    return true;
}

bool Shader::compileShader(
    GLuint shader,
    const std::string& source,
    const std::string& name
)
{
    const char* src = source.c_str();

    glShaderSource(shader, 1, &src, nullptr);
    glCompileShader(shader);

    GLint success = 0;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);

    if (!success)
    {
        char log[1024];
        glGetShaderInfoLog(shader, 1024, nullptr, log);

        std::cerr << "Shader compile failed: "
                  << name << "\n"
                  << log << '\n';

        return false;
    }

    return true;
}

void Shader::setMat4(
    const std::string& name,
    const Mat4& matrix
) const
{
    GLint location =
        glGetUniformLocation(
            m_program,
            name.c_str()
        );

    glUniformMatrix4fv(
        location,
        1,
        GL_FALSE,
        matrix.m
    );
}
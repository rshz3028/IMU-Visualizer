
#pragma once

#include <string>
#include <glad/gl.h>

#include "math3d.h"

class Shader
{
public:
    Shader();
    ~Shader();

    bool loadFromFiles(
        const std::string& vertexPath,
        const std::string& fragmentPath
    );

    void use() const;
    void destroy();

    GLuint id() const;

    void setMat4(
        const std::string& name,
        const Mat4& matrix
    ) const;

private:
    bool readFile(
        const std::string& path,
        std::string& output
    );

    bool compileShader(
        GLuint shader,
        const std::string& source,
        const std::string& name
    );

private:
    GLuint m_program;
};
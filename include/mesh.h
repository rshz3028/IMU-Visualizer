#pragma once

#include <vector>

#include <glad/gl.h>

struct Vertex
{
    float x;
    float y;
    float z;

    float r;
    float g;
    float b;
};  

class Mesh
{
public:
    Mesh();
    ~Mesh();

    bool upload(const std::vector<Vertex>& vertices);

    void draw(GLenum primitive = GL_TRIANGLES) const;

    void destroy();

    Mesh(const Mesh&) = delete;
    Mesh& operator=(const Mesh&) = delete;

    Mesh(Mesh&& other) noexcept;
    Mesh& operator=(Mesh&& other) noexcept;

private:
    GLuint m_vao;
    GLuint m_vbo;

    GLsizei m_vertexCount;


};
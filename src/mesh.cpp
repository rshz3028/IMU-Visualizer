#include "mesh.h"

Mesh::Mesh()
    : m_vao(0),
      m_vbo(0),
      m_vertexCount(0)
{
}

Mesh::~Mesh()
{
    destroy();
}

Mesh::Mesh(Mesh&& other) noexcept
    : m_vao(other.m_vao),
      m_vbo(other.m_vbo),
      m_vertexCount(other.m_vertexCount)
{
    other.m_vao = 0;
    other.m_vbo = 0;
    other.m_vertexCount = 0;
}

Mesh& Mesh::operator=(Mesh&& other) noexcept
{
    if (this != &other)
    {
        destroy();

        m_vao = other.m_vao;
        m_vbo = other.m_vbo;
        m_vertexCount = other.m_vertexCount;

        other.m_vao = 0;
        other.m_vbo = 0;
        other.m_vertexCount = 0;
    }

    return *this;
}

bool Mesh::upload(const std::vector<Vertex>& vertices)
{
    destroy();

    m_vertexCount = static_cast<GLsizei>(vertices.size());

    glGenVertexArrays(1, &m_vao);
    glGenBuffers(1, &m_vbo);

    glBindVertexArray(m_vao);
    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);

    glBufferData(
        GL_ARRAY_BUFFER,
        vertices.size() * sizeof(Vertex),
        vertices.data(),
        GL_STATIC_DRAW
    );

    // Position
    glVertexAttribPointer(
        0,
        3,
        GL_FLOAT,
        GL_FALSE,
        sizeof(Vertex),
        (void*)0
    );
    glEnableVertexAttribArray(0);

    // Color
    glVertexAttribPointer(
        1,
        3,
        GL_FLOAT,
        GL_FALSE,
        sizeof(Vertex),
        (void*)(3 * sizeof(float))
    );
    glEnableVertexAttribArray(1);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    return true;
}

void Mesh::draw(GLenum primitive) const
{
    if (m_vao == 0 || m_vertexCount == 0)
        return;

    glBindVertexArray(m_vao);
    glDrawArrays(
        primitive,
        0,
        m_vertexCount
    );
    glBindVertexArray(0);
}

void Mesh::destroy()
{
    if (m_vbo != 0)
    {
        glDeleteBuffers(1, &m_vbo);
        m_vbo = 0;
    }

    if (m_vao != 0)
    {
        glDeleteVertexArrays(1, &m_vao);
        m_vao = 0;
    }

    m_vertexCount = 0;
}
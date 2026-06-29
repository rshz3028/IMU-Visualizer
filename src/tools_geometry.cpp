#include "tools_geometry.h"

std::vector<Vertex> ToolsGeometry::createGrid(
    int halfSize,
    float spacing)
{
    std::vector<Vertex> vertices;

    const float extent = halfSize * spacing;

    for (int i = -halfSize; i <= halfSize; i++)
    {
        float p = i * spacing;

        vertices.push_back({
            -extent, -0.01f, p,
            0.45f, 0.45f, 0.45f
        });

        vertices.push_back({
             extent, -0.01f, p,
             0.45f, 0.45f, 0.45f
        });

        vertices.push_back({
            p, -0.01f, -extent,
            0.45f, 0.45f, 0.45f
        });

        vertices.push_back({
            p, -0.01f, extent,
            0.45f, 0.45f, 0.45f
        });
    }

    return vertices;
}

std::vector<Vertex> ToolsGeometry::createAxes(
    float length)
{
    std::vector<Vertex> vertices;

    // X

    vertices.push_back({
        0.0f, 0.0f, 0.0f,
        1.0f, 0.0f, 0.0f
    });

    vertices.push_back({
        length, 0.0f, 0.0f,
        1.0f, 0.0f, 0.0f
    });

    // Y

    vertices.push_back({
        0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f
    });

    vertices.push_back({
        0.0f, length, 0.0f,
        0.0f, 1.0f, 0.0f
    });

    // Z

    vertices.push_back({
        0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f
    });

    vertices.push_back({
        0.0f, 0.0f, length,
        0.0f, 0.0f, 1.0f
    });

    return vertices;
}

std::vector<Vertex> ToolsGeometry::createCube(float size)
{
    std::vector<Vertex> vertices;

    float h = size * 0.5f;

    auto add = [&](float x, float y, float z)
    {
        vertices.push_back({
            x, y, z,
            0.8f, 0.8f, 0.8f
        });
    };

    // Front
    add(-h,-h, h); add( h,-h, h); add( h, h, h);
    add(-h,-h, h); add( h, h, h); add(-h, h, h);

    // Back
    add( h,-h,-h); add(-h,-h,-h); add(-h, h,-h);
    add( h,-h,-h); add(-h, h,-h); add( h, h,-h);

    // Left
    add(-h,-h,-h); add(-h,-h, h); add(-h, h, h);
    add(-h,-h,-h); add(-h, h, h); add(-h, h,-h);

    // Right
    add( h,-h, h); add( h,-h,-h); add( h, h,-h);
    add( h,-h, h); add( h, h,-h); add( h, h, h);

    // Top
    add(-h, h, h); add( h, h, h); add( h, h,-h);
    add(-h, h, h); add( h, h,-h); add(-h, h,-h);

    // Bottom
    add(-h,-h,-h); add( h,-h,-h); add( h,-h, h);
    add(-h,-h,-h); add( h,-h, h); add(-h,-h, h);

    return vertices;
}
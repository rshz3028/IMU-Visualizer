#pragma once

#include <vector>

#include "mesh.h"

namespace ToolsGeometry
{
    std::vector<Vertex> createGrid(
        int halfSize,
        float spacing
    );

    std::vector<Vertex> createAxes(
        float length
    );

    std::vector<Vertex> createCube(
        float size
    );

    std::vector<Vertex> createCylinder(
        float radius,
        float height,
        int segments
    );

}
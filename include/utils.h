#pragma once

#include <filesystem>

namespace Utils
{
    std::filesystem::path executableDirectory();

    std::filesystem::path assetPath(
        const std::string& relativePath
    );
}
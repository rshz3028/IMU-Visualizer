#include "utils.h"

#include <SDL3/SDL.h>

namespace Utils
{
    std::filesystem::path executableDirectory()
    {
        return std::filesystem::path(
            SDL_GetBasePath()
        );
    }

    std::filesystem::path assetPath(
        const std::string& relativePath
    )
    {
        return executableDirectory() /
               "assets" /
               relativePath;
    }
}
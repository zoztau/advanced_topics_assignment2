#pragma once

#include <drone_mapper/Types.h>

#include <filesystem>

namespace drone_mapper {

class ConfigLoader {
public:
    [[nodiscard]] static std::filesystem::path resolveInputPath(const std::filesystem::path& path);
    [[nodiscard]] static types::SimulationCompositionData loadComposition(
        const std::filesystem::path& composition_file);
};

} // namespace drone_mapper

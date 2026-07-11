#pragma once

#include <drone_mapper/IMap3D.h>
#include <drone_mapper/Types.h>

#include <filesystem>
#include <optional>
#include <vector>

namespace drone_mapper {

class MapsComparison {
public:
    [[nodiscard]] static std::vector<double> compare(const IMap3D& origin,
                                                     const std::vector<IMap3D*> targets); //currently should work with at least 1 target
    [[nodiscard]] static double compareSingle(const IMap3D& origin, const IMap3D& target);
    [[nodiscard]] static double compareFiles(
        const std::filesystem::path& origin_map,
        const std::filesystem::path& target_map,
        const std::optional<std::filesystem::path>& comparison_config = std::nullopt);
};

} // namespace drone_mapper

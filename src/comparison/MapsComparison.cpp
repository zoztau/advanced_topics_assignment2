#include <drone_mapper/MapsComparison.h>

#include <drone_mapper/Map3DImpl.h>

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace drone_mapper {
namespace {

[[nodiscard]] double xCm(XLength value) {
    return value.force_numerical_value_in(cm);
}

[[nodiscard]] double yCm(YLength value) {
    return value.force_numerical_value_in(cm);
}

[[nodiscard]] double zCm(ZLength value) {
    return value.force_numerical_value_in(cm);
}

[[nodiscard]] double lengthCm(PhysicalLength value) {
    return value.force_numerical_value_in(cm);
}

[[nodiscard]] XLength xLengthCm(double value) {
    return value * x_extent[cm];
}

[[nodiscard]] YLength yLengthCm(double value) {
    return value * y_extent[cm];
}

[[nodiscard]] ZLength zLengthCm(double value) {
    return value * z_extent[cm];
}

[[nodiscard]] const YAML::Node optionalRoot(const YAML::Node& root) {
    return root["comparison_config"] ? root["comparison_config"] : root;
}

[[nodiscard]] const YAML::Node requireNode(const YAML::Node& node,
                                           const char* key,
                                           const std::filesystem::path& path) {
    const YAML::Node child = node[key];
    if (!child) {
        throw std::runtime_error("Missing comparison YAML key '" + std::string{key} + "' in " + path.string());
    }
    return child;
}

struct ParsedMapConfig {
    types::MapConfig config{};
    bool has_boundaries = false;
};

struct FileComparisonConfig {
    types::MapConfig original{};
    types::MapConfig target{};
    types::MapConfig domain{};
    bool has_domain_boundaries = false;
};

[[nodiscard]] const YAML::Node firstNode(const YAML::Node& node, const char* first, const char* second) {
    const YAML::Node first_node = node[first];
    return first_node ? first_node : node[second];
}

[[nodiscard]] double parseResolutionCm(const YAML::Node& node,
                                       const std::filesystem::path& path,
                                       bool required) {
    const YAML::Node value = firstNode(node, "map_res_cm", "map_resolution_cm");
    if (!value) {
        if (required) {
            throw std::runtime_error("Missing comparison YAML key 'map_res_cm' in " + path.string());
        }
        return 1.0;
    }
    return value.as<double>();
}

[[nodiscard]] Position3D parseOffset(const YAML::Node& node) {
    if (!node) {
        return {};
    }

    return Position3D{
        xLengthCm(node["x_offset"] ? node["x_offset"].as<double>() : 0.0),
        yLengthCm(node["y_offset"] ? node["y_offset"].as<double>() : 0.0),
        zLengthCm(node["height_offset"] ? node["height_offset"].as<double>() : 0.0),
    };
}

[[nodiscard]] types::MappingBounds parseBoundaries(const YAML::Node& node,
                                                   const std::filesystem::path& path) {
    const YAML::Node x_boundary = requireNode(node, "x_boundary", path);
    const YAML::Node y_boundary = requireNode(node, "y_boundary", path);
    const YAML::Node height_boundary = requireNode(node, "height_boundary", path);

    return types::MappingBounds{
        xLengthCm(requireNode(x_boundary, "min_cm", path).as<double>()),
        xLengthCm(requireNode(x_boundary, "max_cm", path).as<double>()),
        yLengthCm(requireNode(y_boundary, "min_cm", path).as<double>()),
        yLengthCm(requireNode(y_boundary, "max_cm", path).as<double>()),
        zLengthCm(requireNode(height_boundary, "min_cm", path).as<double>()),
        zLengthCm(requireNode(height_boundary, "max_cm", path).as<double>()),
    };
}

[[nodiscard]] ParsedMapConfig parseMapConfig(const YAML::Node& node,
                                             const std::filesystem::path& path,
                                             bool resolution_required) {
    ParsedMapConfig parsed{};
    parsed.config.resolution = parseResolutionCm(node, path, resolution_required) * cm;
    parsed.config.offset = parseOffset(firstNode(node, "map_offset", "map_axes_offset"));

    const YAML::Node boundaries = firstNode(node, "map_boundaries", "boundaries");
    if (boundaries) {
        parsed.config.boundaries = parseBoundaries(boundaries, path);
        parsed.has_boundaries = true;
    }

    return parsed;
}

[[nodiscard]] types::MappingBounds intersectBounds(const types::MappingBounds& left,
                                                   const types::MappingBounds& right) {
    return types::MappingBounds{
        xLengthCm(std::max(xCm(left.min_x), xCm(right.min_x))),
        xLengthCm(std::min(xCm(left.max_x), xCm(right.max_x))),
        yLengthCm(std::max(yCm(left.min_y), yCm(right.min_y))),
        yLengthCm(std::min(yCm(left.max_y), yCm(right.max_y))),
        zLengthCm(std::max(zCm(left.min_height), zCm(right.min_height))),
        zLengthCm(std::min(zCm(left.max_height), zCm(right.max_height))),
    };
}

[[nodiscard]] FileComparisonConfig loadComparisonConfig(const std::filesystem::path& path) {
    const YAML::Node root = optionalRoot(YAML::LoadFile(path.string()));
    FileComparisonConfig comparison{};

    if (root["original"] || root["target"]) {
        const ParsedMapConfig original = parseMapConfig(requireNode(root, "original", path), path, true);
        const ParsedMapConfig target = parseMapConfig(requireNode(root, "target", path), path, true);
        comparison.original = original.config;
        comparison.target = target.config;
        comparison.domain.resolution = original.config.resolution;

        if (original.has_boundaries && target.has_boundaries) {
            comparison.domain.boundaries = intersectBounds(original.config.boundaries, target.config.boundaries);
            comparison.has_domain_boundaries = true;
        } else if (original.has_boundaries) {
            comparison.domain.boundaries = original.config.boundaries;
            comparison.has_domain_boundaries = true;
        } else if (target.has_boundaries) {
            comparison.domain.boundaries = target.config.boundaries;
            comparison.has_domain_boundaries = true;
        }

        return comparison;
    }

    const ParsedMapConfig shared = parseMapConfig(root, path, false);
    comparison.original = shared.config;
    comparison.target = shared.config;
    comparison.domain = shared.config;
    comparison.has_domain_boundaries = shared.has_boundaries;
    return comparison;
}

[[nodiscard]] std::size_t countAlong(double min_value, double max_value, double resolution) {
    if (resolution <= 0.0 || max_value <= min_value) {
        return 0;
    }
    return static_cast<std::size_t>(std::ceil((max_value - min_value) / resolution));
}

[[nodiscard]] bool hasUsableBounds(const types::MapConfig& config) {
    return xCm(config.boundaries.max_x) > xCm(config.boundaries.min_x) &&
           yCm(config.boundaries.max_y) > yCm(config.boundaries.min_y) &&
           zCm(config.boundaries.max_height) > zCm(config.boundaries.min_height);
}

[[nodiscard]] types::MapConfig completeComparisonDomain(types::MapConfig config,
                                                        bool has_configured_boundaries,
                                                        const IMap3D& origin,
                                                        const IMap3D& target) {
    const types::MapConfig origin_config = origin.getMapConfig();
    const types::MapConfig target_config = target.getMapConfig();
    if (lengthCm(config.resolution) <= 0.0) {
        config.resolution = origin_config.resolution;
    }
    if (!has_configured_boundaries || !hasUsableBounds(config)) {
        config.boundaries = intersectBounds(origin_config.boundaries, target_config.boundaries);
    }
    return config;
}

[[nodiscard]] double compareWithinDomain(const IMap3D& origin,
                                         const IMap3D& target,
                                         const types::MapConfig& config) {
    const double resolution = lengthCm(config.resolution);
    const std::size_t x_count = countAlong(xCm(config.boundaries.min_x),
                                           xCm(config.boundaries.max_x),
                                           resolution);
    const std::size_t y_count = countAlong(yCm(config.boundaries.min_y),
                                           yCm(config.boundaries.max_y),
                                           resolution);
    const std::size_t z_count = countAlong(zCm(config.boundaries.min_height),
                                           zCm(config.boundaries.max_height),
                                           resolution);
    const std::size_t total = x_count * y_count * z_count;
    if (total == 0) {
        return -1.0;
    }

    std::size_t matches = 0;
    for (std::size_t x = 0; x < x_count; ++x) {
        for (std::size_t y = 0; y < y_count; ++y) {
            for (std::size_t z = 0; z < z_count; ++z) {
                const Position3D position{
                    (xCm(config.boundaries.min_x) + static_cast<double>(x) * resolution) * x_extent[cm],
                    (yCm(config.boundaries.min_y) + static_cast<double>(y) * resolution) * y_extent[cm],
                    (zCm(config.boundaries.min_height) + static_cast<double>(z) * resolution) * z_extent[cm],
                };
                if (origin.atVoxel(position) == target.atVoxel(position)) {
                    ++matches;
                }
            }
        }
    }

    return 100.0 * static_cast<double>(matches) / static_cast<double>(total);
}

} // namespace

std::vector<double> MapsComparison::compare(const IMap3D& original,
                               const std::vector<IMap3D*> targets) {
    std::vector<double> scores;
    scores.reserve(targets.size());
    for (const IMap3D* target : targets) {
        scores.push_back(target == nullptr ? -1.0 : compareSingle(original, *target));
    }
    return scores;
}

double MapsComparison::compareSingle(const IMap3D& origin, const IMap3D& target) {
    return compareWithinDomain(origin, target, origin.getMapConfig());
}

double MapsComparison::compareFiles(
    const std::filesystem::path& origin_map,
    const std::filesystem::path& target_map,
    const std::optional<std::filesystem::path>& comparison_config) {
    if (comparison_config.has_value()) {
        const FileComparisonConfig config = loadComparisonConfig(*comparison_config);
        const std::unique_ptr<Map3DImpl> origin =
            Map3DImpl::loadFromNpyFile(origin_map, config.original.resolution, config.original.offset);
        const std::unique_ptr<Map3DImpl> target =
            Map3DImpl::loadFromNpyFile(target_map, config.target.resolution, config.target.offset);
        return compareWithinDomain(
            *origin,
            *target,
            completeComparisonDomain(config.domain, config.has_domain_boundaries, *origin, *target));
    }

    const std::unique_ptr<Map3DImpl> origin =
        Map3DImpl::loadFromNpyFile(origin_map, 1.0 * cm, Position3D{});
    const std::unique_ptr<Map3DImpl> target =
        Map3DImpl::loadFromNpyFile(target_map, 1.0 * cm, Position3D{});
    if (origin->dimensions() != target->dimensions()) {
        throw std::runtime_error("Map dimensions differ and no comparison_config was provided.");
    }
    return compareSingle(*origin, *target);
}

} // namespace drone_mapper

#include <drone_mapper/ConfigLoader.h>

#include <yaml-cpp/yaml.h>

#include <stdexcept>
#include <string>
#include <utility>

namespace drone_mapper {
namespace {

[[nodiscard]] PhysicalLength physicalCm(double value) {
    return value * cm;
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

[[nodiscard]] HorizontalAngle horizontalDeg(double value) {
    return value * horizontal_angle[deg];
}

[[nodiscard]] const YAML::Node requireNode(const YAML::Node& node,
                                           const char* key,
                                           const std::filesystem::path& path) {
    const YAML::Node child = node[key];
    if (!child) {
        throw std::runtime_error("Missing YAML key '" + std::string{key} + "' in " + path.string());
    }
    return child;
}

[[nodiscard]] std::filesystem::path resolveRelative(const std::filesystem::path& base,
                                                    const std::filesystem::path& path) {
    if (path.is_absolute()) {
        return path.lexically_normal();
    }
    return (base / path).lexically_normal();
}

[[nodiscard]] types::MappingBounds parseBounds(const YAML::Node& node,
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

[[nodiscard]] types::DroneConfigData loadDroneConfig(const std::filesystem::path& path) {
    const YAML::Node root = YAML::LoadFile(path.string());
    const YAML::Node config = requireNode(root, "drone_config", path);
    const double dimensions_cm = requireNode(config, "dimensions_cm", path).as<double>();
    return types::DroneConfigData{
        physicalCm(dimensions_cm / 2.0),
        horizontalDeg(requireNode(config, "max_rotate_deg", path).as<double>()),
        physicalCm(requireNode(config, "max_advance_cm", path).as<double>()),
        physicalCm(requireNode(config, "max_elevate_cm", path).as<double>()),
    };
}

[[nodiscard]] types::LidarConfigData loadLidarConfig(const std::filesystem::path& path) {
    const YAML::Node root = YAML::LoadFile(path.string());
    const YAML::Node config = requireNode(root, "lidar_config", path);
    return types::LidarConfigData{
        physicalCm(requireNode(config, "z_min_cm", path).as<double>()),
        physicalCm(requireNode(config, "z_max_cm", path).as<double>()),
        physicalCm(requireNode(config, "d_cm", path).as<double>()),
        requireNode(config, "fov_circles", path).as<std::size_t>(),
    };
}

[[nodiscard]] types::MissionConfigData loadMissionConfig(const std::filesystem::path& path) {
    const YAML::Node root = YAML::LoadFile(path.string());
    const YAML::Node config = requireNode(root, "mission_config", path);
    const YAML::Node factor = config["output_mapping_resolution_factor"];
    return types::MissionConfigData{
        requireNode(config, "max_steps", path).as<std::size_t>(),
        physicalCm(requireNode(config, "gps_resolution_cm", path).as<double>()),
        factor ? factor.as<double>() : 1.0,
        parseBounds(requireNode(config, "boundaries", path), path),
    };
}

[[nodiscard]] Position3D parsePosition(const YAML::Node& node, const std::filesystem::path& path) {
    return Position3D{
        xLengthCm(requireNode(node, "x_cm", path).as<double>()),
        yLengthCm(requireNode(node, "y_cm", path).as<double>()),
        zLengthCm(requireNode(node, "height_cm", path).as<double>()),
    };
}

[[nodiscard]] Position3D parseOffset(const YAML::Node& node, const std::filesystem::path& path) {
    return Position3D{
        xLengthCm(requireNode(node, "x_offset", path).as<double>()),
        yLengthCm(requireNode(node, "y_offset", path).as<double>()),
        zLengthCm(requireNode(node, "height_offset", path).as<double>()),
    };
}

[[nodiscard]] types::SimulationConfigData loadSimulationConfig(const std::filesystem::path& path,
                                                               const std::filesystem::path& base_dir) {
    const YAML::Node root = YAML::LoadFile(path.string());
    const YAML::Node config = requireNode(root, "simulation_config", path);
    return types::SimulationConfigData{
        resolveRelative(base_dir, requireNode(config, "map_filename", path).as<std::string>()),
        physicalCm(requireNode(config, "map_resolution_cm", path).as<double>()),
        parseOffset(requireNode(config, "map_axes_offset", path), path),
        parsePosition(requireNode(config, "initial_drone_position", path), path),
        horizontalDeg(requireNode(config, "initial_angle_deg", path).as<double>()),
    };
}

} // namespace

std::filesystem::path ConfigLoader::resolveInputPath(const std::filesystem::path& path) {
    if (path.is_absolute()) {
        return path.lexically_normal();
    }
    return (std::filesystem::current_path() / path).lexically_normal();
}

types::SimulationCompositionData ConfigLoader::loadComposition(const std::filesystem::path& composition_file) {
    const std::filesystem::path resolved_composition = resolveInputPath(composition_file);
    const std::filesystem::path base_dir = resolved_composition.parent_path();
    const YAML::Node root = YAML::LoadFile(resolved_composition.string());
    const YAML::Node composition = requireNode(root, "simulation_compositions", resolved_composition);

    types::SimulationCompositionData data;
    data.composition_file = composition_file;

    const YAML::Node simulations = requireNode(composition, "simulations", resolved_composition);
    for (const YAML::Node& simulation_group : simulations) {
        const std::filesystem::path simulation_config_file =
            requireNode(simulation_group, "simulation_config", resolved_composition).as<std::string>();
        const std::filesystem::path simulation_path = resolveRelative(base_dir, simulation_config_file);
        types::SimulationConfigEntry simulation_entry{
            simulation_config_file,
            loadSimulationConfig(simulation_path, base_dir),
            {},
        };
        std::vector<types::MissionConfigData> missions;
        for (const YAML::Node& mission_path_node : requireNode(simulation_group, "mission_configs", resolved_composition)) {
            const std::filesystem::path mission_config_file = mission_path_node.as<std::string>();
            types::MissionConfigData mission_config = loadMissionConfig(resolveRelative(base_dir, mission_config_file));
            simulation_entry.missions.push_back(types::MissionConfigEntry{mission_config_file, mission_config});
            missions.push_back(std::move(mission_config));
        }
        data.simulation_mission_groups.emplace_back(simulation_entry.config, std::move(missions));
        data.simulation_entries.push_back(std::move(simulation_entry));
    }

    for (const YAML::Node& drone_path_node : requireNode(composition, "drone_configs", resolved_composition)) {
        const std::filesystem::path drone_config_file = drone_path_node.as<std::string>();
        types::DroneConfigData drone_config = loadDroneConfig(resolveRelative(base_dir, drone_config_file));
        data.drone_entries.push_back(types::DroneConfigEntry{drone_config_file, drone_config});
        data.drones.push_back(std::move(drone_config));
    }

    for (const YAML::Node& lidar_path_node : requireNode(composition, "lidar_configs", resolved_composition)) {
        const std::filesystem::path lidar_config_file = lidar_path_node.as<std::string>();
        types::LidarConfigData lidar_config = loadLidarConfig(resolveRelative(base_dir, lidar_config_file));
        data.lidar_entries.push_back(types::LidarConfigEntry{lidar_config_file, lidar_config});
        data.lidars.push_back(std::move(lidar_config));
    }

    return data;
}

} // namespace drone_mapper

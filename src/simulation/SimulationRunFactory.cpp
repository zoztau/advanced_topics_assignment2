#include <drone_mapper/SimulationRunFactory.h>

#include <drone_mapper/DroneControlImpl.h>
#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MappingAlgorithmImpl.h>
#include <drone_mapper/MissionControlImpl.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockLidar.h>
#include <drone_mapper/MockMovement.h>
#include <drone_mapper/SimulationRunImpl.h>

#include <fstream>
#include <memory>
#include <string>
#include <utility>

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

[[nodiscard]] Position3D outputOffsetForBounds(const types::MappingBounds& bounds) {
    return Position3D{
        (-xCm(bounds.min_x)) * x_extent[cm],
        (-yCm(bounds.min_y)) * y_extent[cm],
        (-zCm(bounds.min_height)) * z_extent[cm],
    };
}

void logResolutionIssue(const std::filesystem::path& output_path, const std::string& message) {
    std::filesystem::create_directories(output_path);
    std::ofstream log{output_path / "errors.log", std::ios::app};
    if (log) {
        log << "step=0 code=OUTPUT_RESOLUTION_IGNORED message=\"" << message << "\"\n";
        log.flush();
    }
}

} // namespace

std::unique_ptr<ISimulationRun>
SimulationRunFactory::create(const types::SimulationConfigData& simulation,
                             const types::MissionConfigData& mission,
                             const types::DroneConfigData& drone,
                             const types::LidarConfigData& lidar,
                             const std::filesystem::path& output_path) {
    std::filesystem::create_directories(output_path);

    auto hidden_map = Map3DImpl::loadFromNpyFile(
        simulation.map_filename,
        simulation.map_resolution,
        simulation.map_offset);

    double output_resolution_factor = mission.output_mapping_resolution_factor;
    types::ResolutionRequestStatus resolution_status = types::ResolutionRequestStatus::Accepted;
    if (output_resolution_factor < 1.0) {
        logResolutionIssue(output_path, "output_mapping_resolution_factor is smaller than 1; using factor 1.");
        output_resolution_factor = 1.0;
        resolution_status = types::ResolutionRequestStatus::IgnoredTooSmall;
    }

    types::MapConfig output_map_config{};
    output_map_config.boundaries = mission.mission_bounds;
    output_map_config.offset = outputOffsetForBounds(mission.mission_bounds);
    output_map_config.resolution = output_resolution_factor * mission.gps_resolution;
    auto output_map = Map3DImpl::createEmpty(output_map_config);

    auto gps = std::make_unique<MockGPS>(
        simulation.initial_drone_position,
        Orientation{simulation.initial_angle, 0.0 * altitude_angle[deg]},
        mission.gps_resolution);
    auto movement = std::make_unique<MockMovement>(*gps, *hidden_map, drone.radius);
    auto lidar_impl = std::make_unique<MockLidar>(lidar, *hidden_map, *gps);
    auto mapping_algorithm = std::make_unique<MappingAlgorithmImpl>(mission, lidar, drone, *output_map);

    auto drone_control = std::make_unique<DroneControlImpl>(
        drone,
        mission,
        *lidar_impl,
        *gps,
        *movement,
        *output_map,
        *mapping_algorithm);

    const std::filesystem::path output_map_file = output_path / "output_map.npy";
    auto mission_control = std::make_unique<MissionControlImpl>(
        mission,
        drone,
        *hidden_map,
        *output_map,
        *drone_control,
        output_map_file);

    return std::make_unique<SimulationRunImpl>(
        std::move(hidden_map),
        std::move(output_map),
        std::move(gps),
        std::move(movement),
        std::move(lidar_impl),
        std::move(mapping_algorithm),
        std::move(drone_control),
        std::move(mission_control),
        simulation,
        mission,
        resolution_status,
        output_map_file);
}

} // namespace drone_mapper

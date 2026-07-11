#pragma once

#include <drone_mapper/types/DroneTypes.h>
#include <drone_mapper/types/LidarTypes.h>
#include <drone_mapper/types/MapTypes.h>
#include <drone_mapper/types/MissionTypes.h>

#include <filesystem>
#include <string>
#include <tuple>
#include <vector>

namespace drone_mapper::types {

struct SimulationConfigData {
    std::filesystem::path map_filename{};
    PhysicalLength map_resolution{};
    // Changed: added offset so world coordinates can be related to the NPY matrix origin.
    Position3D map_offset{}; // moving the (0,0,0) of the npy matrix to this point
    Position3D initial_drone_position{};
    HorizontalAngle initial_angle{};
};

struct MissionConfigEntry {
    std::filesystem::path config_file{};
    MissionConfigData config{};
};

struct SimulationConfigEntry {
    std::filesystem::path config_file{};
    SimulationConfigData config{};
    std::vector<MissionConfigEntry> missions{};
};

struct DroneConfigEntry {
    std::filesystem::path config_file{};
    DroneConfigData config{};
};

struct LidarConfigEntry {
    std::filesystem::path config_file{};
    LidarConfigData config{};
};

struct SimulationCompositionData {
    std::filesystem::path composition_file;
    std::vector<SimulationConfigEntry> simulation_entries;
    std::vector<DroneConfigEntry> drone_entries;
    std::vector<LidarConfigEntry> lidar_entries;
    // 20.6 - added nested structure to enable correct Composition configuration
    std::vector<std::tuple<SimulationConfigData, std::vector<MissionConfigData>>> simulation_mission_groups;
    std::vector<DroneConfigData> drones;
    std::vector<LidarConfigData> lidars;
};

enum class ResolutionRequestStatus {
    Accepted,
    Ignored,
    IgnoredTooSmall,
};

struct SimulationResult {
    std::filesystem::path simulation_config_file{};
    std::filesystem::path mission_config_file{};
    std::filesystem::path drone_config_file{};
    std::filesystem::path lidar_config_file{};
    // Changed: result now carries the simulation and mission configs that produced this run.
    SimulationConfigData simulation_config{};
    MissionConfigData mission_config{};
    DroneConfigData drone_config{};
    LidarConfigData lidar_config{};
    ResolutionRequestStatus resolution_request_status = ResolutionRequestStatus::Ignored;
    // Changed: renamed to plural because this is a collection of mission-level outcomes.
    std::vector<MissionRunResult> mission_results{};
    // Changed: moved from MissionRunResult because output files are simulation-run metadata.
    std::filesystem::path output_map_file{};
    // Changed: added so reports preserve the offset-aware output map configuration.
    MapConfig output_map_config; 
    // Changed: moved from MissionRunResult because score is computed by comparing full maps.
    double mission_score = 0.0;
};

// Changed: report keeps the run list internally; SimulationManager serializes it as nested YAML.
struct SimulationManagerReport {
    std::filesystem::path composition_file{};
    std::string generated_at_utc{};
    std::string metric{};
    // Changed: concrete tuple type replaces the previous placeholder/typo.
    std::tuple<double, double> score_range{};
    int error_score = -1;
    std::vector<SimulationResult> runs{};
};

} // namespace drone_mapper::types

#include <drone_mapper/SimulationManager.h>

#include <algorithm>
#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace drone_mapper {
namespace {

[[nodiscard]] double lengthCm(PhysicalLength value) {
    return value.force_numerical_value_in(cm);
}

[[nodiscard]] std::string generatedAtUtc() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm utc_time{};
    gmtime_r(&now_time, &utc_time);
    std::ostringstream stream;
    stream << std::put_time(&utc_time, "%Y-%m-%dT%H:%M:%SZ");
    return stream.str();
}

[[nodiscard]] std::string missionStatus(types::MissionRunStatus status) {
    switch (status) {
    case types::MissionRunStatus::Completed:
        return "completed";
    case types::MissionRunStatus::MaxSteps:
        return "max_steps";
    case types::MissionRunStatus::Error:
        return "error";
    }
    return "error";
}

[[nodiscard]] std::string resolutionStatus(types::ResolutionRequestStatus status) {
    switch (status) {
    case types::ResolutionRequestStatus::Accepted:
        return "ACCEPTED";
    case types::ResolutionRequestStatus::Ignored:
        return "IGNORED";
    case types::ResolutionRequestStatus::IgnoredTooSmall:
        return "IGNORED_TOO_SMALL";
    }
    return "IGNORED";
}

[[nodiscard]] std::string runDirectoryName(std::size_t index) {
    std::ostringstream stream;
    stream << "run_" << std::setw(3) << std::setfill('0') << index;
    return stream.str();
}

[[nodiscard]] std::string pathText(const std::filesystem::path& path) {
    return path.string();
}

[[nodiscard]] std::size_t firstMissionSteps(const types::SimulationResult& result) {
    return result.mission_results.empty() ? 0 : result.mission_results.front().steps;
}

[[nodiscard]] std::string firstMissionStatus(const types::SimulationResult& result) {
    return result.mission_results.empty() ? "error" : missionStatus(result.mission_results.front().status);
}

[[nodiscard]] const types::ErrorRef* firstError(const types::SimulationResult& result) {
    if (result.mission_results.empty() || result.mission_results.front().errors.empty()) {
        return nullptr;
    }
    return &result.mission_results.front().errors.front();
}

[[nodiscard]] double outputResolutionCm(const types::SimulationResult& result) {
    const double output_resolution = lengthCm(result.output_map_config.resolution);
    if (output_resolution > 0.0) {
        return output_resolution;
    }

    const double factor = result.mission_config.output_mapping_resolution_factor < 1.0
        ? 1.0
        : result.mission_config.output_mapping_resolution_factor;
    return lengthCm(result.mission_config.gps_resolution) * factor;
}

[[nodiscard]] bool sameSimulation(const types::SimulationResult& result,
                                  const std::filesystem::path& simulation_config_file) {
    return result.simulation_config_file == simulation_config_file;
}

[[nodiscard]] bool sameMission(const types::SimulationResult& result,
                               const std::filesystem::path& simulation_config_file,
                               const std::filesystem::path& mission_config_file) {
    return sameSimulation(result, simulation_config_file) &&
           result.mission_config_file == mission_config_file;
}

[[nodiscard]] std::vector<types::SimulationConfigEntry>
normalizedSimulationEntries(const types::SimulationCompositionData& composition) {
    if (!composition.simulation_entries.empty()) {
        return composition.simulation_entries;
    }

    std::vector<types::SimulationConfigEntry> entries;
    entries.reserve(composition.simulation_mission_groups.size());
    for (const auto& [simulation, missions] : composition.simulation_mission_groups) {
        types::SimulationConfigEntry entry{};
        entry.config = simulation;
        entry.missions.reserve(missions.size());
        for (const types::MissionConfigData& mission : missions) {
            entry.missions.push_back(types::MissionConfigEntry{std::filesystem::path{}, mission});
        }
        entries.push_back(std::move(entry));
    }
    return entries;
}

[[nodiscard]] std::vector<types::DroneConfigEntry>
normalizedDroneEntries(const types::SimulationCompositionData& composition) {
    if (!composition.drone_entries.empty()) {
        return composition.drone_entries;
    }

    std::vector<types::DroneConfigEntry> entries;
    entries.reserve(composition.drones.size());
    for (const types::DroneConfigData& drone : composition.drones) {
        entries.push_back(types::DroneConfigEntry{std::filesystem::path{}, drone});
    }
    return entries;
}

[[nodiscard]] std::vector<types::LidarConfigEntry>
normalizedLidarEntries(const types::SimulationCompositionData& composition) {
    if (!composition.lidar_entries.empty()) {
        return composition.lidar_entries;
    }

    std::vector<types::LidarConfigEntry> entries;
    entries.reserve(composition.lidars.size());
    for (const types::LidarConfigData& lidar : composition.lidars) {
        entries.push_back(types::LidarConfigEntry{std::filesystem::path{}, lidar});
    }
    return entries;
}

void attachRunMetadata(types::SimulationResult& result,
                       const types::SimulationConfigEntry& simulation,
                       const types::MissionConfigEntry& mission,
                       const types::DroneConfigEntry& drone,
                       const types::LidarConfigEntry& lidar) {
    result.simulation_config_file = simulation.config_file;
    result.mission_config_file = mission.config_file;
    result.drone_config_file = drone.config_file;
    result.lidar_config_file = lidar.config_file;
    result.simulation_config = simulation.config;
    result.mission_config = mission.config;
    result.drone_config = drone.config;
    result.lidar_config = lidar.config;
}

void appendManagerErrorLog(const std::filesystem::path& log_path, const std::string& message) {
    if (log_path.has_parent_path()) {
        std::filesystem::create_directories(log_path.parent_path());
    }
    std::ofstream log{log_path, std::ios::app};
    if (log) {
        log << "step=0 code=SIMULATION_MANAGER_ERROR message=\"" << message << "\"\n";
        log.flush();
    }
}

void writeRunYaml(std::ofstream& output, const types::SimulationResult& run) {
    output << "            - drone_config: \"" << pathText(run.drone_config_file) << "\"\n";
    output << "              lidar_config: \"" << pathText(run.lidar_config_file) << "\"\n";
    output << "              status: \"" << firstMissionStatus(run) << "\"\n";
    output << "              steps: " << firstMissionSteps(run) << "\n";
    output << "              score: " << run.mission_score << "\n";

    const types::ErrorRef* error = firstError(run);
    if (error != nullptr) {
        output << "              error_ref:\n";
        output << "                code: \"" << error->code << "\"\n";
        output << "                message: \"" << error->message << "\"\n";
    }
}

void writeReportYaml(const types::SimulationManagerReport& report,
                     const std::filesystem::path& output_path) {
    std::filesystem::create_directories(output_path);
    std::ofstream output{output_path / "simulation_output.yaml"};
    if (!output) {
        throw std::runtime_error("Could not write simulation_output.yaml.");
    }

    const std::size_t total_runs = report.runs.size();
    std::size_t error_runs = 0;
    std::size_t scored_runs = 0;
    double score_sum = 0.0;
    double min_score = std::numeric_limits<double>::max();
    double max_score = std::numeric_limits<double>::lowest();
    for (const types::SimulationResult& run : report.runs) {
        if (run.mission_score < 0.0) {
            ++error_runs;
            continue;
        }
        ++scored_runs;
        score_sum += run.mission_score;
        min_score = std::min(min_score, run.mission_score);
        max_score = std::max(max_score, run.mission_score);
    }

    output << "score_report:\n";
    output << "  composition_file: \"" << pathText(report.composition_file) << "\"\n";
    output << "  generated_at_utc: \"" << report.generated_at_utc << "\"\n";
    output << "  metric: \"" << report.metric << "\"\n";
    output << "  score_range:\n";
    output << "    min: " << std::get<0>(report.score_range) << "\n";
    output << "    max: " << std::get<1>(report.score_range) << "\n";
    output << "  error_score: " << report.error_score << "\n";
    output << "  summary:\n";
    output << "    total_runs: " << total_runs << "\n";
    output << "    scored_runs: " << scored_runs << "\n";
    output << "    error_runs: " << error_runs << "\n";
    output << "    average_score: " << (scored_runs == 0 ? -1.0 : score_sum / static_cast<double>(scored_runs)) << "\n";
    output << "    min_score: " << (scored_runs == 0 ? -1.0 : min_score) << "\n";
    output << "    max_score: " << (scored_runs == 0 ? -1.0 : max_score) << "\n";

    if (report.runs.empty()) {
        output << "  simulations: []\n";
        return;
    }

    output << "  simulations:\n";
    std::size_t run_index = 0;
    while (run_index < report.runs.size()) {
        const std::filesystem::path simulation_config_file = report.runs[run_index].simulation_config_file;
        output << "    - simulation_config: \"" << pathText(simulation_config_file) << "\"\n";
        output << "      missions:\n";

        while (run_index < report.runs.size() &&
               sameSimulation(report.runs[run_index], simulation_config_file)) {
            const std::filesystem::path mission_config_file = report.runs[run_index].mission_config_file;
            const types::SimulationResult& mission_reference = report.runs[run_index];
            output << "        - mission_config: \"" << pathText(mission_config_file) << "\"\n";
            output << "          resolution_cm: " << outputResolutionCm(mission_reference) << "\n";
            output << "          resolution_request_status: \""
                   << resolutionStatus(mission_reference.resolution_request_status) << "\"\n";
            output << "          runs:\n";

            while (run_index < report.runs.size() &&
                   sameMission(report.runs[run_index], simulation_config_file, mission_config_file)) {
                writeRunYaml(output, report.runs[run_index]);
                ++run_index;
            }
        }
    }
}

[[nodiscard]] types::SimulationResult errorResult(const types::SimulationConfigData& simulation,
                                                  const types::MissionConfigData& mission,
                                                  const std::filesystem::path& output_map_file,
                                                  const std::string& message) {
    types::SimulationResult result{};
    result.simulation_config = simulation;
    result.mission_config = mission;
    result.output_map_file = output_map_file;
    result.mission_score = -1.0;
    result.resolution_request_status = types::ResolutionRequestStatus::Ignored;
    result.mission_results.push_back(types::MissionRunResult{
        types::MissionRunStatus::Error,
        0,
        {types::ErrorRef{"SIMULATION_MANAGER_ERROR", message}},
    });
    return result;
}

} // namespace

SimulationManager::SimulationManager(std::unique_ptr<ISimulationRunFactory> run_factory)
    : run_factory_(std::move(run_factory)) {
    if (!run_factory_) {
        throw std::invalid_argument("SimulationManager requires a run factory.");
    }
}

types::SimulationManagerReport SimulationManager::run(const types::SimulationCompositionData& composition,
                                                      const std::filesystem::path& output_path) {
    std::vector<types::SimulationResult> runs;
    const std::filesystem::path results_path = output_path / "output_results";
    std::filesystem::create_directories(results_path);

    const std::vector<types::SimulationConfigEntry> simulations = normalizedSimulationEntries(composition);
    const std::vector<types::DroneConfigEntry> drones = normalizedDroneEntries(composition);
    const std::vector<types::LidarConfigEntry> lidars = normalizedLidarEntries(composition);

    std::size_t run_index = 0;
    for (const types::SimulationConfigEntry& simulation : simulations) {
        for (const types::MissionConfigEntry& mission : simulation.missions) {
            for (const types::DroneConfigEntry& drone : drones) {
                for (const types::LidarConfigEntry& lidar : lidars) {
                    const std::filesystem::path run_output_path = results_path / runDirectoryName(run_index);
                    const std::filesystem::path output_map_file = run_output_path / "output_map.npy";
                    types::SimulationResult result{};
                    try {
                        std::unique_ptr<ISimulationRun> run =
                            run_factory_->create(simulation.config,
                                                 mission.config,
                                                 drone.config,
                                                 lidar.config,
                                                 run_output_path);
                        if (!run) {
                            throw std::runtime_error("Run factory returned null.");
                        }
                        result = run->run();
                    } catch (const std::exception& error) {
                        std::filesystem::create_directories(run_output_path);
                        appendManagerErrorLog(run_output_path / "errors.log", error.what());
                        appendManagerErrorLog(output_path / "errors.log", error.what());
                        result = errorResult(simulation.config, mission.config, output_map_file, error.what());
                    }
                    attachRunMetadata(result, simulation, mission, drone, lidar);
                    runs.push_back(std::move(result));
                    ++run_index;
                }
            }
        }
    }

    types::SimulationManagerReport report{
        composition.composition_file,
        generatedAtUtc(),
        "output_map_accuracy",
        {0.0, 100.0},
        -1,
        std::move(runs),
    };
    writeReportYaml(report, output_path);
    return report;
}

} // namespace drone_mapper

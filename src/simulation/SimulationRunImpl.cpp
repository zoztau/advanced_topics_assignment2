#include <drone_mapper/SimulationRunImpl.h>

#include <drone_mapper/MapsComparison.h>

#include <fstream>
#include <stdexcept>
#include <system_error>
#include <utility>
#include <vector>

namespace drone_mapper {
namespace {

void appendErrorLog(const std::filesystem::path& log_path, const types::ErrorRef& error) {
    if (log_path.has_parent_path()) {
        std::error_code directory_error;
        std::filesystem::create_directories(log_path.parent_path(), directory_error);
        if (directory_error) {
            return;
        }
    }

    std::ofstream log{log_path, std::ios::app};
    if (log) {
        log << "step=0 code=" << error.code
            << " message=\"" << error.message << "\"\n";
        log.flush();
    }
}

} // namespace

SimulationRunImpl::SimulationRunImpl(std::unique_ptr<const IMap3D> hidden_map,
                                     std::unique_ptr<IMutableMap3D> output_map,
                                     std::unique_ptr<IGPS> gps,
                                     std::unique_ptr<IDroneMovement> movement,
                                     std::unique_ptr<ILidar> lidar,
                                     std::unique_ptr<IMappingAlgorithm> mapping_algorithm,
                                     std::unique_ptr<IDroneControl> drone_control,
                                     std::unique_ptr<IMissionControl> mission_control,
                                     types::SimulationConfigData simulation_config,
                                     types::MissionConfigData mission_config,
                                     types::ResolutionRequestStatus resolution_request_status,
                                     std::filesystem::path output_map_file)
    : hidden_map_(std::move(hidden_map)),
      output_map_(std::move(output_map)),
      gps_(std::move(gps)),
      movement_(std::move(movement)),
      lidar_(std::move(lidar)),
      mapping_algorithm_(std::move(mapping_algorithm)),
      drone_control_(std::move(drone_control)),
      mission_control_(std::move(mission_control)),
      simulation_config_(std::move(simulation_config)),
      mission_config_(std::move(mission_config)),
      resolution_request_status_(resolution_request_status),
      output_map_file_(std::move(output_map_file)) {
    if (!hidden_map_ ||
        !output_map_ ||
        !gps_ ||
        !movement_ ||
        !lidar_ ||
        !mapping_algorithm_ ||
        !drone_control_ ||
        !mission_control_) {
        throw std::invalid_argument("SimulationRunImpl requires injected dependencies.");
    }
}

types::SimulationResult SimulationRunImpl::run() {
    types::SimulationResult result{};
    result.simulation_config = simulation_config_;
    result.mission_config = mission_config_;
    result.resolution_request_status = resolution_request_status_;
    result.output_map_file = output_map_file_;
    result.output_map_config = output_map_->getMapConfig();
    result.mission_score = -1.0;

    try {
        types::MissionRunResult mission_result = mission_control_->runMission();
        result.mission_results.push_back(mission_result);
        if (mission_result.status != types::MissionRunStatus::Error) {
            const std::vector<double> scores = MapsComparison::compare(*hidden_map_, {output_map_.get()});
            result.mission_score = scores.empty() ? -1.0 : scores.front();
        }
    } catch (const std::exception& error) {
        const std::size_t completed_steps =
            result.mission_results.empty() ? 0 : result.mission_results.front().steps;
        const types::ErrorRef run_error{"SIMULATION_RUN_ERROR", error.what()};
        appendErrorLog(output_map_file_.parent_path() / "errors.log", run_error);
        result.mission_results.clear();
        result.mission_results.push_back(types::MissionRunResult{
            types::MissionRunStatus::Error,
            completed_steps,
            {run_error},
        });
    }

    return result;
}

} // namespace drone_mapper

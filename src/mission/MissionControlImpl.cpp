#include <drone_mapper/MissionControlImpl.h>

#include <fstream>
#include <string>
#include <utility>

namespace drone_mapper {
namespace {

[[nodiscard]] std::filesystem::path errorLogPath(const std::filesystem::path& output_map_file) {
    return output_map_file.parent_path() / "errors.log";
}

[[nodiscard]] std::filesystem::path aggregateErrorLogPath(const std::filesystem::path& output_map_file) {
    const std::filesystem::path run_dir = output_map_file.parent_path();
    const std::filesystem::path output_results_dir = run_dir.parent_path();
    if (output_results_dir.filename() != "output_results") {
        return {};
    }
    return output_results_dir.parent_path() / "errors.log";
}

void appendErrorLog(const std::filesystem::path& log_path,
                    const types::ErrorRef& error,
                    std::size_t step) {
    if (log_path.empty()) {
        return;
    }
    if (log_path.has_parent_path()) {
        std::filesystem::create_directories(log_path.parent_path());
    }
    std::ofstream log{log_path, std::ios::app};
    if (log) {
        log << "step=" << step
            << " code=" << error.code
            << " message=\"" << error.message << "\"\n";
        log.flush();
    }
}

void logErrorImmediately(const std::filesystem::path& output_map_file,
                         const types::ErrorRef& error,
                         std::size_t step) {
    appendErrorLog(errorLogPath(output_map_file), error, step);
    appendErrorLog(aggregateErrorLogPath(output_map_file), error, step);
}

[[nodiscard]] types::MissionRunResult result(types::MissionRunStatus status,
                                             std::size_t steps,
                                             std::vector<types::ErrorRef> errors = {}) {
    return types::MissionRunResult{status, steps, std::move(errors)};
}

} // namespace

MissionControlImpl::MissionControlImpl(types::MissionConfigData mission,
                                       types::DroneConfigData drone,
                                       const IMap3D& hidden_map,
                                       IMutableMap3D& output_map,
                                       IDroneControl& drone_control,
                                       std::filesystem::path output_map_file)
    : mission_(std::move(mission)),
      drone_(std::move(drone)),
      hidden_map_(hidden_map),
      output_map_(output_map),
      drone_control_(drone_control),
      output_map_file_(std::move(output_map_file)) {}

types::MissionRunResult MissionControlImpl::runMission() {
    const Position3D start_position = drone_control_.state().position;
    if (!hidden_map_.isInBounds(start_position) ||
        hidden_map_.atVoxel(start_position) == types::VoxelOccupancy::Occupied) {
        const types::ErrorRef error{
            "INVALID_INITIAL_POSITION",
            "Initial drone position is outside the hidden map or occupied."};
        logErrorImmediately(output_map_file_, error, 0);
        output_map_.save(output_map_file_);
        return result(types::MissionRunStatus::Error, 0, {error});
    }

    for (std::size_t step = 0; step < mission_.max_steps; ++step) {
        const types::DroneStepResult step_result = drone_control_.step();
        const std::size_t completed_steps = step + 1;

        if (step_result.status == types::DroneStepStatus::Completed) {
            output_map_.save(output_map_file_);
            return result(types::MissionRunStatus::Completed, completed_steps);
        }

        if (step_result.status == types::DroneStepStatus::Error) {
            const types::ErrorRef error{
                "DRONE_STEP_ERROR",
                step_result.message.empty() ? "Drone step failed." : step_result.message};
            logErrorImmediately(output_map_file_, error, completed_steps);
            output_map_.save(output_map_file_);
            return result(types::MissionRunStatus::Error, completed_steps, {error});
        }
    }

    output_map_.save(output_map_file_);
    return result(types::MissionRunStatus::MaxSteps, mission_.max_steps);
}

} // namespace drone_mapper

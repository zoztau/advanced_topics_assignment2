#include <drone_mapper/MissionControlImpl.h>

#include <geometry/VoxelGeometry.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <string>
#include <utility>

namespace drone_mapper {
namespace {

constexpr double kCollisionEpsilon = 1.0e-6;

struct AxisIndexRange {
    int first = 1;
    int last = 0;

    [[nodiscard]] bool empty() const {
        return first > last;
    }
};

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

[[nodiscard]] int dimensionFromSpan(double min_value, double max_value, double resolution) {
    if (resolution <= 0.0 || max_value <= min_value) {
        return 0;
    }
    return static_cast<int>(std::ceil((max_value - min_value) / resolution));
}

[[nodiscard]] AxisIndexRange candidateRange(double center,
                                            double radius,
                                            double map_min,
                                            double map_max,
                                            double resolution) {
    const int dimension = dimensionFromSpan(map_min, map_max, resolution);
    if (dimension == 0) {
        return {};
    }

    const int first = static_cast<int>(std::floor((center - radius - map_min) / resolution)) - 1;
    const int last = static_cast<int>(std::floor((center + radius - map_min) / resolution)) + 1;
    return AxisIndexRange{
        std::max(0, first),
        std::min(dimension - 1, last),
    };
}

[[nodiscard]] bool sphereFitsBounds(const Position3D& center,
                                    double radius,
                                    const types::MappingBounds& bounds) {
    return xCm(center.x) - radius >= xCm(bounds.min_x) &&
           xCm(center.x) + radius < xCm(bounds.max_x) &&
           yCm(center.y) - radius >= yCm(bounds.min_y) &&
           yCm(center.y) + radius < yCm(bounds.max_y) &&
           zCm(center.z) - radius >= zCm(bounds.min_height) &&
           zCm(center.z) + radius < zCm(bounds.max_height);
}

[[nodiscard]] double clippedMidpoint(double voxel_min,
                                     double voxel_max,
                                     double map_min,
                                     double map_max) {
    const double clipped_min = std::max(voxel_min, map_min);
    const double clipped_max = std::min(voxel_max, map_max);
    return clipped_min + (clipped_max - clipped_min) * 0.5;
}

[[nodiscard]] Position3D voxelProbe(const geometry::VoxelAabb& box,
                                    const types::MappingBounds& bounds) {
    return Position3D{
        clippedMidpoint(box.min[0], box.max[0], xCm(bounds.min_x), xCm(bounds.max_x)) * x_extent[cm],
        clippedMidpoint(box.min[1], box.max[1], yCm(bounds.min_y), yCm(bounds.max_y)) * y_extent[cm],
        clippedMidpoint(box.min[2], box.max[2], zCm(bounds.min_height), zCm(bounds.max_height)) *
            z_extent[cm],
    };
}

[[nodiscard]] bool initialSphereIsValid(const Position3D& center,
                                        PhysicalLength drone_radius,
                                        const types::MappingBounds& mission_bounds,
                                        const IMap3D& hidden_map) {
    const double radius = std::max(0.0, lengthCm(drone_radius));
    const types::MapConfig config = hidden_map.getMapConfig();
    const double resolution = lengthCm(config.resolution);
    if (resolution <= 0.0 || !hidden_map.isInBounds(center) ||
        !sphereFitsBounds(center, radius, mission_bounds) ||
        !sphereFitsBounds(center, radius, config.boundaries)) {
        return false;
    }

    const AxisIndexRange x_range = candidateRange(
        xCm(center.x), radius,
        xCm(config.boundaries.min_x), xCm(config.boundaries.max_x), resolution);
    const AxisIndexRange y_range = candidateRange(
        yCm(center.y), radius,
        yCm(config.boundaries.min_y), yCm(config.boundaries.max_y), resolution);
    const AxisIndexRange z_range = candidateRange(
        zCm(center.z), radius,
        zCm(config.boundaries.min_height), zCm(config.boundaries.max_height), resolution);
    if (x_range.empty() || y_range.empty() || z_range.empty()) {
        return false;
    }

    for (int x = x_range.first; x <= x_range.last; ++x) {
        for (int y = y_range.first; y <= y_range.last; ++y) {
            for (int z = z_range.first; z <= z_range.last; ++z) {
                const geometry::VoxelAabb box = geometry::voxelAabb(config, x, y, z);
                if (hidden_map.atVoxel(voxelProbe(box, config.boundaries)) ==
                        types::VoxelOccupancy::Occupied &&
                    geometry::squaredDistanceToVoxelAabb(center, x, y, z, config) <=
                        radius * radius + kCollisionEpsilon) {
                    return false;
                }
            }
        }
    }

    return true;
}

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
    if (!initialSphereIsValid(
            start_position, drone_.radius, mission_.mission_bounds, hidden_map_)) {
        const types::ErrorRef error{
            "INVALID_INITIAL_POSITION",
            "Initial drone sphere is outside mission or hidden-map bounds, or intersects an occupied voxel."};
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

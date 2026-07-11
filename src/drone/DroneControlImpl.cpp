#include <drone_mapper/DroneControlImpl.h>

#include <drone_mapper/ScanResultToVoxels.h>

#include <mp-units/systems/si/math.h>

#include <cmath>
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

[[nodiscard]] double lengthCm(PhysicalLength value) {
    return value.force_numerical_value_in(cm);
}

[[nodiscard]] double angleDeg(HorizontalAngle value) {
    return value.force_numerical_value_in(deg);
}

[[nodiscard]] bool sphereFitsInMission(const Position3D& position,
                                       PhysicalLength radius,
                                       const types::MappingBounds& bounds) {
    const double r = lengthCm(radius);
    return xCm(position.x) - r >= xCm(bounds.min_x) &&
           xCm(position.x) + r < xCm(bounds.max_x) &&
           yCm(position.y) - r >= yCm(bounds.min_y) &&
           yCm(position.y) + r < yCm(bounds.max_y) &&
           zCm(position.z) - r >= zCm(bounds.min_height) &&
           zCm(position.z) + r < zCm(bounds.max_height);
}

[[nodiscard]] Position3D advancedPosition(const Position3D& position,
                                          Orientation heading,
                                          PhysicalLength distance) {
    const double distance_cm = lengthCm(distance);
    const double dx = si::cos(heading.horizontal).force_numerical_value_in(mp::one);
    const double dy = si::sin(heading.horizontal).force_numerical_value_in(mp::one);
    return Position3D{
        position.x + dx * distance_cm * x_extent[cm],
        position.y + dy * distance_cm * y_extent[cm],
        position.z,
    };
}

[[nodiscard]] Position3D elevatedPosition(const Position3D& position, PhysicalLength distance) {
    return Position3D{
        position.x,
        position.y,
        position.z + lengthCm(distance) * z_extent[cm],
    };
}

[[nodiscard]] types::DroneStepResult errorResult(std::string message) {
    return types::DroneStepResult{types::DroneStepStatus::Error, std::move(message)};
}

} // namespace

DroneControlImpl::DroneControlImpl(types::DroneConfigData drone,
                                   types::MissionConfigData mission,
                                   ILidar& lidar,
                                   IGPS& gps,
                                   IDroneMovement& movement,
                                   IMutableMap3D& output_map,
                                   IMappingAlgorithm& mapping_algorithm)
    : drone_(std::move(drone)),
      mission_(std::move(mission)),
      lidar_(lidar),
      gps_(gps),
      movement_(movement),
      output_map_(output_map),
      mapping_algorithm_(mapping_algorithm) {}

types::DroneStepResult DroneControlImpl::step() {
    const types::DroneState current_state = state();
    const types::LidarScanResult* latest_scan = latest_scan_ ? &(*latest_scan_) : nullptr;
    const types::MappingStepCommand command = mapping_algorithm_.nextStep(current_state, latest_scan);

    if (command.status == types::AlgorithmStatus::Finished ||
        command.status == types::AlgorithmStatus::FinishedWithUnmappableVoxels) {
        return types::DroneStepResult{types::DroneStepStatus::Completed, "Mapping algorithm finished."};
    }

    if (command.movement.has_value()) {
        const types::MovementCommand& movement = *command.movement;
        types::MovementResult result{};
        switch (movement.type) {
        case types::MovementCommandType::Hover:
            break;
        case types::MovementCommandType::Rotate:
            if (std::abs(angleDeg(movement.angle)) > angleDeg(drone_.max_rotate)) {
                return errorResult("Rotate command exceeds drone max_rotate.");
            }
            result = movement_.rotate(movement.rotation, movement.angle);
            if (!result) {
                return errorResult(result.message.empty() ? "Rotate command failed." : result.message);
            }
            break;
        case types::MovementCommandType::Advance:
            if (std::abs(lengthCm(movement.distance)) > lengthCm(drone_.max_advance)) {
                return errorResult("Advance command exceeds drone max_advance.");
            }
            if (!sphereFitsInMission(advancedPosition(gps_.position(), gps_.heading(), movement.distance),
                                     drone_.radius,
                                     mission_.mission_bounds)) {
                return errorResult("Advance command would leave mission boundaries.");
            }
            result = movement_.advance(movement.distance);
            if (!result) {
                return errorResult(result.message.empty() ? "Advance command failed." : result.message);
            }
            break;
        case types::MovementCommandType::Elevate:
            if (std::abs(lengthCm(movement.distance)) > lengthCm(drone_.max_elevate)) {
                return errorResult("Elevate command exceeds drone max_elevate.");
            }
            if (!sphereFitsInMission(elevatedPosition(gps_.position(), movement.distance),
                                     drone_.radius,
                                     mission_.mission_bounds)) {
                return errorResult("Elevate command would leave mission boundaries.");
            }
            result = movement_.elevate(movement.distance);
            if (!result) {
                return errorResult(result.message.empty() ? "Elevate command failed." : result.message);
            }
            break;
        }
    }

    if (command.scan_orientation.has_value()) {
        latest_scan_ = lidar_.scan(*command.scan_orientation);
        ScanResultToVoxels::applyToMap(output_map_,
                                       gps_.position(),
                                       gps_.heading(),
                                       *latest_scan_,
                                       lidar_.config());
    }

    ++step_index_;
    return types::DroneStepResult{types::DroneStepStatus::Continue, {}};
}

types::DroneState DroneControlImpl::state() const {
    return types::DroneState{gps_.position(), gps_.heading(), step_index_};
}

} // namespace drone_mapper

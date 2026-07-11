#include <drone_mapper/MappingAlgorithmImpl.h>

#include <drone_mapper/IMap3D.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <optional>
#include <queue>
#include <set>
#include <vector>

namespace drone_mapper {
namespace {

constexpr double kEpsilon = 1.0e-6;
constexpr double kLeanScanResolutionThresholdCm = 15.0;
constexpr std::size_t kConservativeScanCellLimit = 80;
constexpr std::size_t kLeanScanCellLimit = 1000;
constexpr std::size_t kMaxBfsCells = 6000;

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

[[nodiscard]] double horizontalDeg(HorizontalAngle value) {
    return value.force_numerical_value_in(deg);
}

[[nodiscard]] double normalizeDegrees(double angle) {
    while (angle < 0.0) {
        angle += 360.0;
    }
    while (angle >= 360.0) {
        angle -= 360.0;
    }
    return angle;
}

[[nodiscard]] MappingAlgorithmImpl::Direction horizontalDirection(Orientation heading) {
    const double angle = normalizeDegrees(horizontalDeg(heading.horizontal));
    if (angle < 45.0 || angle >= 315.0) {
        return MappingAlgorithmImpl::Direction{1, 0, 0};
    }
    if (angle < 135.0) {
        return MappingAlgorithmImpl::Direction{0, 1, 0};
    }
    if (angle < 225.0) {
        return MappingAlgorithmImpl::Direction{-1, 0, 0};
    }
    return MappingAlgorithmImpl::Direction{0, -1, 0};
}

[[nodiscard]] MappingAlgorithmImpl::Direction leftOf(MappingAlgorithmImpl::Direction direction) {
    return MappingAlgorithmImpl::Direction{-direction.dy, direction.dx, 0};
}

[[nodiscard]] MappingAlgorithmImpl::Direction rightOf(MappingAlgorithmImpl::Direction direction) {
    return MappingAlgorithmImpl::Direction{direction.dy, -direction.dx, 0};
}

[[nodiscard]] double directionHeadingDegrees(const MappingAlgorithmImpl::Direction& direction) {
    if (direction.dx == 0 && direction.dy == 0) {
        return 0.0;
    }
    return normalizeDegrees(std::atan2(static_cast<double>(direction.dy),
                                       static_cast<double>(direction.dx)) * 180.0 / M_PI);
}

[[nodiscard]] Orientation scanOrientationForDirection(const MappingAlgorithmImpl::Direction& direction) {
    const double horizontal = directionHeadingDegrees(direction);
    const double horizontal_length =
        std::hypot(static_cast<double>(direction.dx), static_cast<double>(direction.dy));
    const double altitude = std::atan2(static_cast<double>(direction.dz), horizontal_length) * 180.0 / M_PI;
    return Orientation{horizontal * horizontal_angle[deg], altitude * altitude_angle[deg]};
}

[[nodiscard]] bool sameDirection(const MappingAlgorithmImpl::Direction& left,
                                 const MappingAlgorithmImpl::Direction& right) {
    return left.dx == right.dx && left.dy == right.dy && left.dz == right.dz;
}

void addUniqueDirection(std::vector<MappingAlgorithmImpl::Direction>& directions,
                        MappingAlgorithmImpl::Direction direction) {
    if (direction.dx == 0 && direction.dy == 0 && direction.dz == 0) {
        return;
    }
    const auto exists = std::find_if(directions.begin(), directions.end(), [&](const auto& current) {
        return sameDirection(current, direction);
    });
    if (exists == directions.end()) {
        directions.push_back(direction);
    }
}

[[nodiscard]] types::MovementCommand rotateCommand(types::RotationDirection direction, HorizontalAngle angle) {
    types::MovementCommand command{};
    command.type = types::MovementCommandType::Rotate;
    command.rotation = direction;
    command.angle = angle;
    return command;
}

[[nodiscard]] types::MovementCommand advanceCommand(PhysicalLength distance) {
    types::MovementCommand command{};
    command.type = types::MovementCommandType::Advance;
    command.distance = distance;
    return command;
}

[[nodiscard]] types::MovementCommand elevateCommand(PhysicalLength distance) {
    types::MovementCommand command{};
    command.type = types::MovementCommandType::Elevate;
    command.distance = distance;
    return command;
}

[[nodiscard]] types::MappingStepCommand workingCommand(types::MovementCommand movement) {
    return types::MappingStepCommand{movement, std::nullopt, types::AlgorithmStatus::Working};
}

[[nodiscard]] types::MappingStepCommand finishedCommand() {
    return types::MappingStepCommand{
        std::nullopt,
        std::nullopt,
        types::AlgorithmStatus::FinishedWithUnmappableVoxels,
    };
}

[[nodiscard]] int dimensionFromSpan(double min_value, double max_value, double resolution) {
    if (resolution <= 0.0 || max_value <= min_value) {
        return 1;
    }
    return std::max(1, static_cast<int>(std::ceil((max_value - min_value) / resolution)));
}

[[nodiscard]] std::array<int, 3> dimensionsFromConfig(const types::MapConfig& config) {
    const double resolution = lengthCm(config.resolution);
    return {
        dimensionFromSpan(xCm(config.boundaries.min_x), xCm(config.boundaries.max_x), resolution),
        dimensionFromSpan(yCm(config.boundaries.min_y), yCm(config.boundaries.max_y), resolution),
        dimensionFromSpan(zCm(config.boundaries.min_height), zCm(config.boundaries.max_height), resolution),
    };
}

[[nodiscard]] double squaredDistanceToCellAabb(const Position3D& position,
                                               const MappingAlgorithmImpl::GridIndex& index,
                                               const types::MapConfig& config) {
    const double resolution = lengthCm(config.resolution);
    const double x_min = xCm(config.boundaries.min_x) + static_cast<double>(index.x) * resolution;
    const double y_min = yCm(config.boundaries.min_y) + static_cast<double>(index.y) * resolution;
    const double z_min = zCm(config.boundaries.min_height) + static_cast<double>(index.z) * resolution;
    const double x_max = x_min + resolution;
    const double y_max = y_min + resolution;
    const double z_max = z_min + resolution;

    const double x = xCm(position.x);
    const double y = yCm(position.y);
    const double z = zCm(position.z);
    const double dx = x < x_min ? x_min - x : (x > x_max ? x - x_max : 0.0);
    const double dy = y < y_min ? y_min - y : (y > y_max ? y - y_max : 0.0);
    const double dz = z < z_min ? z_min - z : (z > z_max ? z - z_max : 0.0);
    return dx * dx + dy * dy + dz * dz;
}

} // namespace

bool MappingAlgorithmImpl::usesLeanScanPattern() const {
    return lengthCm(output_map_.getMapConfig().resolution) >= kLeanScanResolutionThresholdCm;
}

std::size_t MappingAlgorithmImpl::maxScannedCells() const {
    if (mission_config_.max_steps == 0) {
        return 1;
    }
    if (usesLeanScanPattern()) {
        const std::size_t step_based_limit = std::max<std::size_t>(1, mission_config_.max_steps / 4);
        return std::min(kLeanScanCellLimit, step_based_limit);
    }

    const std::size_t step_based_limit = std::max<std::size_t>(1, mission_config_.max_steps / 12);
    return std::min(kConservativeScanCellLimit, step_based_limit);
}

MappingAlgorithmImpl::GridIndex MappingAlgorithmImpl::gridFromPosition(const Position3D& position) const {
    const types::MapConfig config = output_map_.getMapConfig();
    const double resolution = lengthCm(config.resolution);
    if (resolution <= 0.0) {
        return {};
    }
    return GridIndex{
        static_cast<int>(std::floor((xCm(position.x) - xCm(config.boundaries.min_x)) / resolution)),
        static_cast<int>(std::floor((yCm(position.y) - yCm(config.boundaries.min_y)) / resolution)),
        static_cast<int>(std::floor((zCm(position.z) - zCm(config.boundaries.min_height)) / resolution)),
    };
}

Position3D MappingAlgorithmImpl::cellCenter(const GridIndex& index) const {
    const types::MapConfig config = output_map_.getMapConfig();
    const double resolution = lengthCm(config.resolution);
    return Position3D{
        (xCm(config.boundaries.min_x) + (static_cast<double>(index.x) + 0.5) * resolution) * x_extent[cm],
        (yCm(config.boundaries.min_y) + (static_cast<double>(index.y) + 0.5) * resolution) * y_extent[cm],
        (zCm(config.boundaries.min_height) + (static_cast<double>(index.z) + 0.5) * resolution) * z_extent[cm],
    };
}

long long MappingAlgorithmImpl::flatKey(const GridIndex& index) const {
    const std::array<int, 3> dimensions = dimensionsFromConfig(output_map_.getMapConfig());
    return (static_cast<long long>(index.x) * dimensions[1] + index.y) * dimensions[2] + index.z;
}

bool MappingAlgorithmImpl::inGrid(const GridIndex& index) const {
    const std::array<int, 3> dimensions = dimensionsFromConfig(output_map_.getMapConfig());
    return index.x >= 0 && index.y >= 0 && index.z >= 0 &&
           index.x < dimensions[0] && index.y < dimensions[1] && index.z < dimensions[2];
}

bool MappingAlgorithmImpl::sameCell(const GridIndex& left, const GridIndex& right) const {
    return left.x == right.x && left.y == right.y && left.z == right.z;
}

bool MappingAlgorithmImpl::isKnownSafe(const GridIndex& index, const GridIndex& current) const {
    if (!inGrid(index)) {
        return false;
    }

    const Position3D center = cellCenter(index);
    if (!output_map_.isInBounds(center)) {
        return false;
    }

    const types::VoxelOccupancy center_value = output_map_.atVoxel(center);
    if (center_value != types::VoxelOccupancy::Empty && !sameCell(index, current)) {
        return false;
    }

    const types::MapConfig config = output_map_.getMapConfig();
    const double resolution = lengthCm(config.resolution);
    const double radius = lengthCm(drone_config_.radius);
    if (resolution <= 0.0) {
        return false;
    }

    const int cell_radius = std::max(0, static_cast<int>(std::ceil(radius / resolution)));
    for (int dx = -cell_radius; dx <= cell_radius; ++dx) {
        for (int dy = -cell_radius; dy <= cell_radius; ++dy) {
            for (int dz = -cell_radius; dz <= cell_radius; ++dz) {
                const GridIndex check{index.x + dx, index.y + dy, index.z + dz};
                if (!inGrid(check)) {
                    if (squaredDistanceToCellAabb(center, check, config) <= radius * radius + kEpsilon) {
                        return false;
                    }
                    continue;
                }

                if (squaredDistanceToCellAabb(center, check, config) > radius * radius + kEpsilon) {
                    continue;
                }

                const types::VoxelOccupancy occupancy = output_map_.atVoxel(cellCenter(check));
                if (occupancy != types::VoxelOccupancy::Empty && !sameCell(check, current)) {
                    return false;
                }
            }
        }
    }

    return true;
}

bool MappingAlgorithmImpl::isKnownSafePath(const GridIndex& from,
                                           const GridIndex& to,
                                           const GridIndex& current) const {
    if (!isKnownSafe(to, current)) {
        return false;
    }

    const Position3D from_center = cellCenter(from);
    const Position3D to_center = cellCenter(to);
    const double dx = xCm(to_center.x) - xCm(from_center.x);
    const double dy = yCm(to_center.y) - yCm(from_center.y);
    const double dz = zCm(to_center.z) - zCm(from_center.z);
    const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    const double resolution = lengthCm(output_map_.getMapConfig().resolution);
    const int samples = std::max(1, static_cast<int>(std::ceil(distance / std::max(resolution * 0.5, 1.0))));

    for (int sample = 1; sample <= samples; ++sample) {
        const double t = static_cast<double>(sample) / static_cast<double>(samples);
        const Position3D position{
            (xCm(from_center.x) + t * dx) * x_extent[cm],
            (yCm(from_center.y) + t * dy) * y_extent[cm],
            (zCm(from_center.z) + t * dz) * z_extent[cm],
        };
        const GridIndex sample_index = gridFromPosition(position);
        if (!isKnownSafe(sample_index, current)) {
            return false;
        }
    }

    return true;
}

double MappingAlgorithmImpl::coverageScore(const GridIndex& index) const {
    if (!inGrid(index)) {
        return 0.0;
    }

    double score = 0.0;
    const int kCoverageRadius = usesLeanScanPattern() ? 3 : 2;
    for (int dx = -kCoverageRadius; dx <= kCoverageRadius; ++dx) {
        for (int dy = -kCoverageRadius; dy <= kCoverageRadius; ++dy) {
            for (int dz = -kCoverageRadius; dz <= kCoverageRadius; ++dz) {
                const GridIndex candidate{index.x + dx, index.y + dy, index.z + dz};
                if (!inGrid(candidate)) {
                    continue;
                }
                const types::VoxelOccupancy occupancy = output_map_.atVoxel(cellCenter(candidate));
                if (occupancy == types::VoxelOccupancy::Unmapped) {
                    const double distance = std::sqrt(static_cast<double>(dx * dx + dy * dy + dz * dz));
                    score += 1.0 / (1.0 + distance);
                }
            }
        }
    }

    if (isFrontier(index)) {
        score += usesLeanScanPattern() ? 8.0 : 5.0;
    }
    return score;
}

double MappingAlgorithmImpl::clearanceScore(const GridIndex& index) const {
    double score = 0.0;
    for (const GridIndex& neighbor : neighbors(index)) {
        const types::VoxelOccupancy occupancy = output_map_.atVoxel(cellCenter(neighbor));
        if (occupancy == types::VoxelOccupancy::Empty) {
            score += 1.0;
        }
    }
    return score;
}

double MappingAlgorithmImpl::scanPotentialScore(const GridIndex& current,
                                                const Direction& direction) const {
    if (direction.dx == 0 && direction.dy == 0 && direction.dz == 0) {
        return 0.0;
    }

    const double resolution = lengthCm(output_map_.getMapConfig().resolution);
    if (resolution <= 0.0) {
        return 0.0;
    }

    const int max_cells = std::max(1, static_cast<int>(std::ceil(lengthCm(lidar_config_.z_max) / resolution)));
    double score = 0.0;
    for (int step = 1; step <= max_cells; ++step) {
        const GridIndex sample{
            current.x + direction.dx * step,
            current.y + direction.dy * step,
            current.z + direction.dz * step,
        };
        if (!inGrid(sample)) {
            break;
        }

        const types::VoxelOccupancy occupancy = output_map_.atVoxel(cellCenter(sample));
        const double distance_weight = 1.0 / (1.0 + 0.12 * static_cast<double>(step));
        if (occupancy == types::VoxelOccupancy::Unmapped) {
            score += distance_weight;
        } else if (occupancy == types::VoxelOccupancy::PotentiallyOccupied) {
            score += 0.35 * distance_weight;
        }
    }
    return score;
}

bool MappingAlgorithmImpl::isFrontier(const GridIndex& index) const {
    if (!inGrid(index) || output_map_.atVoxel(cellCenter(index)) != types::VoxelOccupancy::Empty) {
        return false;
    }

    for (const GridIndex& neighbor : neighbors(index)) {
        if (output_map_.atVoxel(cellCenter(neighbor)) == types::VoxelOccupancy::Unmapped) {
            return true;
        }
    }
    return false;
}

std::vector<MappingAlgorithmImpl::GridIndex> MappingAlgorithmImpl::neighbors(const GridIndex& index) const {
    static constexpr std::array<Direction, 6> kDirections{
        Direction{1, 0, 0},
        Direction{-1, 0, 0},
        Direction{0, 1, 0},
        Direction{0, -1, 0},
        Direction{0, 0, 1},
        Direction{0, 0, -1},
    };

    std::vector<GridIndex> result;
    result.reserve(kDirections.size());
    for (const Direction& direction : kDirections) {
        const GridIndex candidate{index.x + direction.dx, index.y + direction.dy, index.z + direction.dz};
        if (inGrid(candidate)) {
            result.push_back(candidate);
        }
    }
    return result;
}

std::optional<MappingAlgorithmImpl::GridIndex>
MappingAlgorithmImpl::chooseLocalMove(const GridIndex& current, Orientation heading) const {
    const Direction forward = horizontalDirection(heading);
    const Direction left = leftOf(forward);
    const Direction right = rightOf(forward);
    const std::array<Direction, 6> directions{
        forward,
        left,
        right,
        Direction{-forward.dx, -forward.dy, 0},
        Direction{0, 0, 1},
        Direction{0, 0, -1},
    };

    std::optional<GridIndex> best;
    double best_score = -1.0;
    for (const Direction& direction : directions) {
        const GridIndex candidate{current.x + direction.dx, current.y + direction.dy, current.z + direction.dz};
        if (!isKnownSafePath(current, candidate, current)) {
            continue;
        }
        if (visited_cells_.contains(flatKey(candidate))) {
            continue;
        }
        const double score = coverageScore(candidate) + 0.25 * clearanceScore(candidate);
        if (score > best_score) {
            best_score = score;
            best = candidate;
        }
    }

    return best;
}

std::vector<MappingAlgorithmImpl::GridIndex>
MappingAlgorithmImpl::pathToQueuedBranchTarget(const GridIndex& current) const {
    std::set<long long> active_branch_targets;
    for (const GridIndex& target : branch_targets_) {
        const long long key = flatKey(target);
        if (!visited_cells_.contains(key) && isKnownSafe(target, current)) {
            active_branch_targets.insert(key);
        }
    }
    if (active_branch_targets.empty()) {
        return {};
    }

    std::queue<GridIndex> frontier;
    std::set<long long> visited;
    std::map<long long, GridIndex> parent;
    frontier.push(current);
    visited.insert(flatKey(current));

    while (!frontier.empty()) {
        const GridIndex node = frontier.front();
        frontier.pop();
        const long long key = flatKey(node);

        if (!sameCell(node, current) && active_branch_targets.contains(key)) {
            std::vector<GridIndex> path;
            GridIndex step = node;
            path.push_back(step);
            while (!sameCell(step, current)) {
                const auto parent_iter = parent.find(flatKey(step));
                if (parent_iter == parent.end()) {
                    return {};
                }
                step = parent_iter->second;
                path.push_back(step);
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (const GridIndex& neighbor : neighbors(node)) {
            const long long neighbor_key = flatKey(neighbor);
            if (visited.contains(neighbor_key) || !isKnownSafePath(node, neighbor, current)) {
                continue;
            }
            visited.insert(neighbor_key);
            parent.emplace(neighbor_key, node);
            frontier.push(neighbor);
        }
    }

    return {};
}

std::vector<MappingAlgorithmImpl::GridIndex>
MappingAlgorithmImpl::pathToBestCoverageTarget(const GridIndex& current) const {
    std::queue<GridIndex> frontier;
    std::set<long long> visited;
    std::map<long long, GridIndex> parent;
    std::map<long long, int> distance_from_start;
    frontier.push(current);
    const long long start_key = flatKey(current);
    visited.insert(start_key);
    distance_from_start.emplace(start_key, 0);

    std::optional<GridIndex> best;
    double best_score = 0.0;
    std::size_t expanded = 0;

    while (!frontier.empty() && expanded < kMaxBfsCells) {
        const GridIndex node = frontier.front();
        frontier.pop();
        ++expanded;
        const long long node_key = flatKey(node);
        const int distance = distance_from_start.at(node_key);

        const double branch_bonus =
            branch_target_keys_.contains(node_key) && !visited_cells_.contains(node_key) ? 25.0 : 0.0;
        const double score =
            coverageScore(node) + 1.25 * clearanceScore(node) + branch_bonus - 0.35 * static_cast<double>(distance);
        if (!sameCell(node, current) && score > best_score) {
            best_score = score;
            best = node;
        }

        for (const GridIndex& neighbor : neighbors(node)) {
            const long long key = flatKey(neighbor);
            if (visited.contains(key) || !isKnownSafePath(node, neighbor, current)) {
                continue;
            }
            visited.insert(key);
            parent.emplace(key, node);
            distance_from_start.emplace(key, distance + 1);
            frontier.push(neighbor);
        }
    }

    if (!best.has_value()) {
        return {};
    }

    std::vector<GridIndex> path;
    GridIndex node = *best;
    path.push_back(node);
    while (!sameCell(node, current)) {
        const auto parent_iter = parent.find(flatKey(node));
        if (parent_iter == parent.end()) {
            return {};
        }
        node = parent_iter->second;
        path.push_back(node);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

std::optional<types::MappingStepCommand> MappingAlgorithmImpl::commandTowardTarget(
    const types::DroneState& state,
    const GridIndex& current,
    const GridIndex& target) {
    if (sameCell(current, target)) {
        pending_target_.reset();
        return std::nullopt;
    }

    const double resolution = lengthCm(output_map_.getMapConfig().resolution);
    if (resolution <= 0.0) {
        pending_target_.reset();
        return finishedCommand();
    }

    const Position3D target_center = cellCenter(target);
    if (target.z != current.z) {
        const double z_delta = zCm(target_center.z) - zCm(state.position.z);
        if (std::abs(z_delta) <= 1.0) {
            pending_target_.reset();
            return std::nullopt;
        }

        const double max_elevate = lengthCm(drone_config_.max_elevate);
        if (max_elevate <= kEpsilon) {
            pending_target_.reset();
            return finishedCommand();
        }

        const double step_distance = std::clamp(z_delta, -max_elevate, max_elevate);
        if (std::abs(z_delta) <= max_elevate + kEpsilon) {
            pending_target_.reset();
        }
        return workingCommand(elevateCommand(step_distance * cm));
    }

    const double x_delta = xCm(target_center.x) - xCm(state.position.x);
    const double y_delta = yCm(target_center.y) - yCm(state.position.y);
    const double horizontal_distance = std::hypot(x_delta, y_delta);
    if (horizontal_distance <= 1.0) {
        pending_target_.reset();
        return std::nullopt;
    }

    const double target_heading = normalizeDegrees(std::atan2(y_delta, x_delta) * 180.0 / M_PI);
    double delta = normalizeDegrees(target_heading - horizontalDeg(state.heading.horizontal));
    if (delta > 180.0) {
        delta -= 360.0;
    }

    if (std::abs(delta) > 1.0) {
        const double rotate_limit = std::max(0.0, horizontalDeg(drone_config_.max_rotate));
        if (rotate_limit <= kEpsilon) {
            pending_target_.reset();
            return finishedCommand();
        }
        const double step_angle = std::min(std::abs(delta), rotate_limit);
        const types::RotationDirection rotate =
            delta >= 0.0 ? types::RotationDirection::Left : types::RotationDirection::Right;
        return workingCommand(rotateCommand(rotate, step_angle * horizontal_angle[deg]));
    }

    const double max_advance = lengthCm(drone_config_.max_advance);
    if (max_advance <= kEpsilon) {
        pending_target_.reset();
        return finishedCommand();
    }

    const double step_distance = std::min(horizontal_distance, max_advance);
    if (horizontal_distance <= max_advance + kEpsilon) {
        pending_target_.reset();
    }
    return workingCommand(advanceCommand(step_distance * cm));
}

bool MappingAlgorithmImpl::movementReachesTarget(const types::DroneState& state,
                                                 const GridIndex& current,
                                                 const GridIndex& target,
                                                 const types::MappingStepCommand& command) const {
    if (!command.movement.has_value()) {
        return false;
    }

    const types::MovementCommand& movement = *command.movement;
    if (movement.type == types::MovementCommandType::Elevate) {
        if (target.z == current.z) {
            return false;
        }
        const double z_delta = zCm(cellCenter(target).z) - zCm(state.position.z);
        return std::abs(z_delta) <= lengthCm(drone_config_.max_elevate) + kEpsilon;
    }

    if (movement.type != types::MovementCommandType::Advance || target.z != current.z) {
        return false;
    }

    const Position3D target_center = cellCenter(target);
    const double x_delta = xCm(target_center.x) - xCm(state.position.x);
    const double y_delta = yCm(target_center.y) - yCm(state.position.y);
    const double horizontal_distance = std::hypot(x_delta, y_delta);
    if (horizontal_distance > lengthCm(drone_config_.max_advance) + kEpsilon) {
        return false;
    }

    const double target_heading = normalizeDegrees(std::atan2(y_delta, x_delta) * 180.0 / M_PI);
    double delta = normalizeDegrees(target_heading - horizontalDeg(state.heading.horizontal));
    if (delta > 180.0) {
        delta -= 360.0;
    }
    return std::abs(delta) <= 1.0;
}

std::optional<MappingAlgorithmImpl::Direction> MappingAlgorithmImpl::takeFirstScanForCell(
    const GridIndex& current,
    Orientation heading) {
    const long long key = flatKey(current);
    if (scanned_cells_.contains(key) || scanned_cells_.size() >= maxScannedCells()) {
        return std::nullopt;
    }

    const Direction forward = horizontalDirection(heading);
    const Direction left = leftOf(forward);
    const Direction right = rightOf(forward);
    std::vector<Direction> directions;
    directions.reserve(usesLeanScanPattern() ? 6U : 10U);
    addUniqueDirection(directions, forward);
    addUniqueDirection(directions, left);
    addUniqueDirection(directions, right);
    addUniqueDirection(directions, Direction{-forward.dx, -forward.dy, 0});
    addUniqueDirection(directions, Direction{0, 0, 1});
    addUniqueDirection(directions, Direction{0, 0, -1});
    if (usesLeanScanPattern()) {
        addUniqueDirection(directions, Direction{forward.dx + left.dx, forward.dy + left.dy, 0});
        addUniqueDirection(directions, Direction{forward.dx + right.dx, forward.dy + right.dy, 0});
    } else {
        addUniqueDirection(directions, Direction{forward.dx + left.dx, forward.dy + left.dy, 0});
        addUniqueDirection(directions, Direction{forward.dx + right.dx, forward.dy + right.dy, 0});
        addUniqueDirection(directions, Direction{-forward.dx + left.dx, -forward.dy + left.dy, 0});
        addUniqueDirection(directions, Direction{-forward.dx + right.dx, -forward.dy + right.dy, 0});
    }

    if (directions.empty()) {
        return std::nullopt;
    }

    std::stable_sort(directions.begin(), directions.end(), [&](const Direction& left_direction,
                                                               const Direction& right_direction) {
        return scanPotentialScore(current, left_direction) > scanPotentialScore(current, right_direction);
    });

    for (std::size_t index = 1; index < directions.size(); ++index) {
        pending_scans_.push_back(directions[index]);
    }
    scanned_cells_.insert(key);
    return directions.front();
}

void MappingAlgorithmImpl::rememberBranchTargets(const GridIndex& current) {
    std::vector<GridIndex> safe_unvisited_neighbors;
    for (const GridIndex& neighbor : neighbors(current)) {
        const long long key = flatKey(neighbor);
        if (!visited_cells_.contains(key) && isKnownSafePath(current, neighbor, current)) {
            safe_unvisited_neighbors.push_back(neighbor);
        }
    }

    if (safe_unvisited_neighbors.size() < 2U) {
        return;
    }

    for (const GridIndex& target : safe_unvisited_neighbors) {
        const long long key = flatKey(target);
        if (branch_target_keys_.insert(key).second) {
            branch_targets_.push_back(target);
        }
    }
}

void MappingAlgorithmImpl::enqueueScansForCell(const GridIndex& current, Orientation heading) {
    if (const auto first_direction = takeFirstScanForCell(current, heading)) {
        pending_scans_.push_front(*first_direction);
    }
}

types::MappingStepCommand MappingAlgorithmImpl::nextStep(const types::DroneState& state,
                                                         const types::LidarScanResult* latest_scan) {
    const bool has_latest_scan = latest_scan != nullptr;
    (void)has_latest_scan;

    const GridIndex current = gridFromPosition(state.position);
    if (!inGrid(current)) {
        return finishedCommand();
    }

    if (mission_config_.max_steps > 0 && state.step_index + 1 >= mission_config_.max_steps) {
        return finishedCommand();
    }

    visited_cells_.insert(flatKey(current));

    if (pending_target_.has_value()) {
        if (auto command = commandTowardTarget(state, current, *pending_target_)) {
            return *command;
        }
    }

    if (pending_scans_.empty()) {
        enqueueScansForCell(current, state.heading);
    }

    if (!pending_scans_.empty()) {
        const Direction direction = pending_scans_.front();
        pending_scans_.pop_front();
        return types::MappingStepCommand{
            std::nullopt,
            scanOrientationForDirection(direction),
            types::AlgorithmStatus::Working,
        };
    }

    rememberBranchTargets(current);

    if (scanned_cells_.size() >= maxScannedCells()) {
        return finishedCommand();
    }

    if (auto local_move = chooseLocalMove(current, state.heading)) {
        pending_target_ = *local_move;
        if (auto command = commandTowardTarget(state, current, *pending_target_)) {
            if (movementReachesTarget(state, current, *local_move, *command)) {
                if (const auto first_direction = takeFirstScanForCell(*local_move, state.heading)) {
                    command->scan_orientation = scanOrientationForDirection(*first_direction);
                }
            }
            return *command;
        }
    }

    const std::vector<GridIndex> branch_path = pathToQueuedBranchTarget(current);
    if (branch_path.size() > 1U) {
        pending_target_ = branch_path[1];
        if (auto command = commandTowardTarget(state, current, *pending_target_)) {
            if (movementReachesTarget(state, current, branch_path[1], *command)) {
                if (const auto first_direction = takeFirstScanForCell(branch_path[1], state.heading)) {
                    command->scan_orientation = scanOrientationForDirection(*first_direction);
                }
            }
            return *command;
        }
    }

    const std::vector<GridIndex> path = pathToBestCoverageTarget(current);
    if (path.size() > 1U) {
        pending_target_ = path[1];
        if (auto command = commandTowardTarget(state, current, *pending_target_)) {
            if (movementReachesTarget(state, current, path[1], *command)) {
                if (const auto first_direction = takeFirstScanForCell(path[1], state.heading)) {
                    command->scan_orientation = scanOrientationForDirection(*first_direction);
                }
            }
            return *command;
        }
    }

    return finishedCommand();
}

} // namespace drone_mapper

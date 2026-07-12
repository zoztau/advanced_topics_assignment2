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
constexpr std::size_t kMaxAdaptiveScansWithoutStepLimit = 512;
constexpr double kRayKeyScale = 1.0e9;
constexpr double kScanDistanceWeightDecay = 0.12;
constexpr double kMinimumUsefulPotentialFraction = 0.05;

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

[[nodiscard]] double altitudeDeg(AltitudeAngle value) {
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

[[nodiscard]] std::size_t beamsOnCircle(std::size_t circle_index) {
    std::size_t count = 1;
    for (std::size_t circle = 0; circle < circle_index; ++circle) {
        count *= 4;
    }
    return count;
}

[[nodiscard]] double directionHeadingDegrees(const MappingAlgorithmImpl::Direction& direction) {
    if (direction.dx == 0 && direction.dy == 0) {
        return 0.0;
    }
    return normalizeDegrees(std::atan2(static_cast<double>(direction.dy),
                                       static_cast<double>(direction.dx)) * 180.0 / M_PI);
}

[[nodiscard]] Orientation scanOrientationForDirection(const MappingAlgorithmImpl::Direction& direction,
                                                      Orientation heading) {
    const double horizontal = normalizeDegrees(
        directionHeadingDegrees(direction) - horizontalDeg(heading.horizontal));
    const double horizontal_length =
        std::hypot(static_cast<double>(direction.dx), static_cast<double>(direction.dy));
    const double altitude =
        std::atan2(static_cast<double>(direction.dz), horizontal_length) * 180.0 / M_PI -
        altitudeDeg(heading.altitude);
    return Orientation{horizontal * horizontal_angle[deg], altitude * altitude_angle[deg]};
}

[[nodiscard]] Orientation scanOrientationToward(const Position3D& target,
                                                const types::DroneState& state) {
    const double dx = xCm(target.x) - xCm(state.position.x);
    const double dy = yCm(target.y) - yCm(state.position.y);
    const double dz = zCm(target.z) - zCm(state.position.z);
    const double horizontal_length = std::hypot(dx, dy);
    const double horizontal = normalizeDegrees(
        std::atan2(dy, dx) * 180.0 / M_PI - horizontalDeg(state.heading.horizontal));
    const double altitude =
        std::atan2(dz, horizontal_length) * 180.0 / M_PI - altitudeDeg(state.heading.altitude);
    return Orientation{horizontal * horizontal_angle[deg], altitude * altitude_angle[deg]};
}

[[nodiscard]] std::optional<std::tuple<long long, long long, long long>> normalizedRayKey(
    const Position3D& target,
    const Position3D& origin) {
    const double dx = xCm(target.x) - xCm(origin.x);
    const double dy = yCm(target.y) - yCm(origin.y);
    const double dz = zCm(target.z) - zCm(origin.z);
    const double length = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (length <= kEpsilon) {
        return std::nullopt;
    }
    return std::make_tuple(
        std::llround(kRayKeyScale * dx / length),
        std::llround(kRayKeyScale * dy / length),
        std::llround(kRayKeyScale * dz / length));
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

types::MappingStepCommand MappingAlgorithmImpl::finishedCommand() const {
    const std::array<int, 3> dimensions = dimensionsFromConfig(output_map_.getMapConfig());
    types::AlgorithmStatus status = types::AlgorithmStatus::Finished;

    for (int x = 0; x < dimensions[0]; ++x) {
        for (int y = 0; y < dimensions[1]; ++y) {
            for (int z = 0; z < dimensions[2]; ++z) {
                const types::VoxelOccupancy occupancy =
                    output_map_.atVoxel(cellCenter(GridIndex{x, y, z}));
                if (occupancy == types::VoxelOccupancy::Unmapped ||
                    occupancy == types::VoxelOccupancy::PotentiallyOccupied) {
                    status = types::AlgorithmStatus::FinishedWithUnmappableVoxels;
                    return types::MappingStepCommand{std::nullopt, std::nullopt, status};
                }
            }
        }
    }

    return types::MappingStepCommand{std::nullopt, std::nullopt, status};
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

MappingAlgorithmImpl::SafetyAnalysis MappingAlgorithmImpl::analyzeKnownSafety(
    const GridIndex& index,
    const GridIndex& current,
    bool collect_all_unresolved) const {
    SafetyAnalysis analysis{};
    if (!inGrid(index)) {
        analysis.hard_blocked = true;
        return analysis;
    }

    const Position3D center = cellCenter(index);
    if (!output_map_.isInBounds(center)) {
        analysis.hard_blocked = true;
        return analysis;
    }

    const types::MapConfig config = output_map_.getMapConfig();
    const double resolution = lengthCm(config.resolution);
    const double radius = std::max(0.0, lengthCm(drone_config_.radius));
    if (resolution <= 0.0) {
        analysis.hard_blocked = true;
        return analysis;
    }

    const int cell_radius = std::max(0, static_cast<int>(std::ceil(radius / resolution)));
    for (int dx = -cell_radius; dx <= cell_radius; ++dx) {
        for (int dy = -cell_radius; dy <= cell_radius; ++dy) {
            for (int dz = -cell_radius; dz <= cell_radius; ++dz) {
                const GridIndex check{index.x + dx, index.y + dy, index.z + dz};
                if (!inGrid(check)) {
                    if (squaredDistanceToCellAabb(center, check, config) <= radius * radius + kEpsilon) {
                        analysis.hard_blocked = true;
                        return analysis;
                    }
                    continue;
                }

                if (squaredDistanceToCellAabb(center, check, config) > radius * radius + kEpsilon) {
                    continue;
                }

                const types::VoxelOccupancy occupancy = output_map_.atVoxel(cellCenter(check));
                if (occupancy == types::VoxelOccupancy::Empty || sameCell(check, current)) {
                    continue;
                }
                if (occupancy == types::VoxelOccupancy::Unmapped ||
                    occupancy == types::VoxelOccupancy::PotentiallyOccupied) {
                    // Another observation can turn either state into Empty, so
                    // these voxels are actionable rather than permanent blocks.
                    analysis.unresolved.push_back(check);
                    if (!collect_all_unresolved) {
                        return analysis;
                    }
                    continue;
                }
                analysis.hard_blocked = true;
                return analysis;
            }
        }
    }

    return analysis;
}

MappingAlgorithmImpl::SafetyAnalysis MappingAlgorithmImpl::analyzeKnownSafePath(
    const GridIndex& from,
    const GridIndex& to,
    const GridIndex& current,
    bool collect_all_unresolved) const {
    SafetyAnalysis result{};
    std::set<long long> unresolved_keys;
    const auto merge = [&](const SafetyAnalysis& analysis) {
        result.hard_blocked = result.hard_blocked || analysis.hard_blocked;
        for (const GridIndex& blocker : analysis.unresolved) {
            const long long key = flatKey(blocker);
            if (unresolved_keys.insert(key).second) {
                result.unresolved.push_back(blocker);
            }
        }
    };

    const Position3D from_center = cellCenter(from);
    const Position3D to_center = cellCenter(to);
    const double dx = xCm(to_center.x) - xCm(from_center.x);
    const double dy = yCm(to_center.y) - yCm(from_center.y);
    const double dz = zCm(to_center.z) - zCm(from_center.z);
    const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    const double resolution = lengthCm(output_map_.getMapConfig().resolution);
    const int samples = std::max(1, static_cast<int>(std::ceil(distance / std::max(resolution * 0.5, 1.0))));

    std::set<long long> analyzed_indices;
    for (int sample = 1; sample <= samples; ++sample) {
        const double t = static_cast<double>(sample) / static_cast<double>(samples);
        const Position3D position{
            (xCm(from_center.x) + t * dx) * x_extent[cm],
            (yCm(from_center.y) + t * dy) * y_extent[cm],
            (zCm(from_center.z) + t * dz) * z_extent[cm],
        };
        const GridIndex sample_index = gridFromPosition(position);
        if (!inGrid(sample_index)) {
            result.hard_blocked = true;
            return result;
        }
        if (!analyzed_indices.insert(flatKey(sample_index)).second) {
            continue;
        }
        merge(analyzeKnownSafety(sample_index, current, collect_all_unresolved));
        if (result.hard_blocked || (!collect_all_unresolved && !result.unresolved.empty())) {
            return result;
        }
    }

    return result;
}

bool MappingAlgorithmImpl::isKnownSafe(const GridIndex& index, const GridIndex& current) const {
    return analyzeKnownSafety(index, current, false).safe();
}

bool MappingAlgorithmImpl::isKnownSafePath(const GridIndex& from,
                                           const GridIndex& to,
                                           const GridIndex& current) const {
    return analyzeKnownSafePath(from, to, current, false).safe();
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

double MappingAlgorithmImpl::scanPotentialScore(const Position3D& origin,
                                                const Direction& direction) const {
    if (direction.dx == 0 && direction.dy == 0 && direction.dz == 0) {
        return 0.0;
    }

    const double resolution = lengthCm(output_map_.getMapConfig().resolution);
    if (resolution <= 0.0) {
        return 0.0;
    }

    const double lidar_range = lengthCm(lidar_config_.z_max);
    if (lidar_range <= 0.0 || lidar_config_.fov_circles == 0) {
        return 0.0;
    }

    const double horizontal_length =
        std::hypot(static_cast<double>(direction.dx), static_cast<double>(direction.dy));
    const double center_horizontal =
        std::atan2(static_cast<double>(direction.dy), static_cast<double>(direction.dx));
    const double center_altitude =
        std::atan2(static_cast<double>(direction.dz), horizontal_length);
    const double sample_step = 0.5 * resolution;
    const double z_min = lengthCm(lidar_config_.z_min);
    const double circle_spacing = lengthCm(lidar_config_.d);

    std::set<long long> scored_voxels;
    double score = 0.0;
    const auto score_beam = [&](double horizontal, double altitude) {
        const double cos_altitude = std::cos(altitude);
        const double ray_x = cos_altitude * std::cos(horizontal);
        const double ray_y = cos_altitude * std::sin(horizontal);
        const double ray_z = std::sin(altitude);

        for (double distance = sample_step; distance <= lidar_range + kEpsilon;
             distance += sample_step) {
            const Position3D sample_position{
                origin.x + ray_x * distance * x_extent[cm],
                origin.y + ray_y * distance * y_extent[cm],
                origin.z + ray_z * distance * z_extent[cm],
            };
            const GridIndex sample = gridFromPosition(sample_position);
            if (!inGrid(sample)) {
                break;
            }

            const types::VoxelOccupancy occupancy = output_map_.atVoxel(cellCenter(sample));
            if (occupancy == types::VoxelOccupancy::Occupied) {
                break;
            }
            if (!scored_voxels.insert(flatKey(sample)).second) {
                continue;
            }

            const double cell_distance = distance / resolution;
            const double distance_weight =
                1.0 / (1.0 + kScanDistanceWeightDecay * cell_distance);
            if (occupancy == types::VoxelOccupancy::Unmapped) {
                score += distance_weight;
            } else if (occupancy == types::VoxelOccupancy::PotentiallyOccupied) {
                score += 0.35 * distance_weight;
            }
        }
    };

    score_beam(center_horizontal, center_altitude);
    for (std::size_t circle = 1; circle < lidar_config_.fov_circles; ++circle) {
        const std::size_t beam_count = beamsOnCircle(circle);
        const double radius = static_cast<double>(circle) * circle_spacing;
        for (std::size_t beam = 0; beam < beam_count; ++beam) {
            const double theta =
                2.0 * M_PI * static_cast<double>(beam) / static_cast<double>(beam_count);
            const double horizontal_offset = radius * std::cos(theta);
            const double altitude_offset = radius * std::sin(theta);
            const double horizontal_delta = std::atan2(horizontal_offset, z_min);
            const double altitude_delta = std::atan2(altitude_offset, z_min);
            score_beam(center_horizontal + horizontal_delta,
                       center_altitude + altitude_delta);
        }
    }

    return score;
}

double MappingAlgorithmImpl::minimumUsefulScanPotential() const {
    const double resolution = lengthCm(output_map_.getMapConfig().resolution);
    if (resolution <= 0.0) {
        return 1.0;
    }

    const int max_cells =
        std::max(1, static_cast<int>(std::ceil(lengthCm(lidar_config_.z_max) / resolution)));
    return kMinimumUsefulPotentialFraction /
           (1.0 + kScanDistanceWeightDecay * static_cast<double>(max_cells));
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

std::optional<Orientation> MappingAlgorithmImpl::takeNextScanForCell(
    const GridIndex& current,
    const types::DroneState& state) {
    const long long key = flatKey(current);
    if ((!scanned_cells_.contains(key) && scanned_cells_.size() >= maxScannedCells()) ||
        lidar_config_.fov_circles == 0 || lengthCm(lidar_config_.z_max) <= 0.0) {
        return std::nullopt;
    }

    const Direction forward = horizontalDirection(state.heading);
    const Direction left = leftOf(forward);
    const Direction right = rightOf(forward);
    const std::array<Direction, 6> base_directions{
        forward,
        left,
        right,
        Direction{-forward.dx, -forward.dy, 0},
        Direction{0, 0, 1},
        Direction{0, 0, -1},
    };

    // Rebuild priorities from the updated map and issue only the best scan.
    std::vector<std::pair<Direction, double>> useful_base_directions;
    const double minimum_potential = minimumUsefulScanPotential();
    for (const Direction& direction : base_directions) {
        const auto attempt = std::make_tuple(key, direction.dx, direction.dy, direction.dz);
        if (attempted_base_scans_.contains(attempt)) {
            continue;
        }

        const double potential = scanPotentialScore(state.position, direction);
        if (potential + kEpsilon >= minimum_potential) {
            useful_base_directions.emplace_back(direction, potential);
        }
    }

    std::stable_sort(useful_base_directions.begin(),
                     useful_base_directions.end(),
                     [](const auto& left_direction, const auto& right_direction) {
        return left_direction.second > right_direction.second;
    });

    if (!useful_base_directions.empty()) {
        const Direction direction = useful_base_directions.front().first;
        attempted_base_scans_.insert(std::make_tuple(key, direction.dx, direction.dy, direction.dz));
        scanned_cells_.insert(key);
        return scanOrientationForDirection(direction, state.heading);
    }

    std::size_t& adaptive_scan_count = adaptive_scan_counts_[key];
    if (mission_config_.max_steps == 0 &&
        adaptive_scan_count >= kMaxAdaptiveScansWithoutStepLimit) {
        return std::nullopt;
    }

    const std::array<Direction, 6> candidate_directions{
        forward,
        left,
        right,
        Direction{-forward.dx, -forward.dy, 0},
        Direction{0, 0, 1},
        Direction{0, 0, -1},
    };
    const types::MapConfig config = output_map_.getMapConfig();
    const double lidar_range = lengthCm(lidar_config_.z_max);
    const double lidar_range_squared = lidar_range * lidar_range;
    std::size_t& candidate_cursor = adaptive_candidate_cursors_[key];
    candidate_cursor %= candidate_directions.size();

    for (std::size_t offset = 0; offset < candidate_directions.size(); ++offset) {
        const std::size_t candidate_index = (candidate_cursor + offset) % candidate_directions.size();
        const Direction& direction = candidate_directions[candidate_index];
        const GridIndex candidate{
            current.x + direction.dx,
            current.y + direction.dy,
            current.z + direction.dz,
        };
        const SafetyAnalysis analysis = analyzeKnownSafePath(current, candidate, current);
        if (analysis.safe() || analysis.hard_blocked) {
            continue;
        }

        std::vector<GridIndex> blockers = analysis.unresolved;
        std::stable_sort(blockers.begin(), blockers.end(), [&](const GridIndex& left_blocker,
                                                               const GridIndex& right_blocker) {
            return squaredDistanceToCellAabb(state.position, left_blocker, config) <
                   squaredDistanceToCellAabb(state.position, right_blocker, config);
        });

        for (const GridIndex& blocker : blockers) {
            if (squaredDistanceToCellAabb(state.position, blocker, config) >
                lidar_range_squared + kEpsilon) {
                continue;
            }

            const Position3D target = cellCenter(blocker);
            const auto direction_key = normalizedRayKey(target, state.position);
            if (!direction_key.has_value()) {
                continue;
            }
            const auto [ray_x, ray_y, ray_z] = *direction_key;
            const auto attempt = std::make_tuple(key, ray_x, ray_y, ray_z);
            if (attempted_adaptive_rays_.contains(attempt)) {
                continue;
            }

            attempted_adaptive_rays_.insert(attempt);
            ++adaptive_scan_count;
            candidate_cursor = (candidate_index + 1U) % candidate_directions.size();
            scanned_cells_.insert(key);
            return scanOrientationToward(target, state);
        }
    }

    return std::nullopt;
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

types::MappingStepCommand MappingAlgorithmImpl::nextStep(const types::DroneState& state,
                                                         const types::LidarScanResult* latest_scan) {
    (void)latest_scan;

    const GridIndex current = gridFromPosition(state.position);
    if (!inGrid(current)) {
        return finishedCommand();
    }

    visited_cells_.insert(flatKey(current));

    if (pending_target_.has_value()) {
        const GridIndex target = *pending_target_;
        if (auto command = commandTowardTarget(state, current, target)) {
            return *command;
        }
    }

    rememberBranchTargets(current);

    // DroneControl applies each requested scan before calling nextStep again.
    // Reconsider movement first so a useful scan immediately stops scanning.
    if (auto local_move = chooseLocalMove(current, state.heading)) {
        pending_target_ = *local_move;
        if (auto command = commandTowardTarget(state, current, *local_move)) {
            return *command;
        }
    }

    if (const auto scan_orientation = takeNextScanForCell(current, state)) {
        return types::MappingStepCommand{
            std::nullopt,
            *scan_orientation,
            types::AlgorithmStatus::Working,
        };
    }

    const std::vector<GridIndex> branch_path = pathToQueuedBranchTarget(current);
    if (branch_path.size() > 1U) {
        pending_target_ = branch_path[1];
        if (auto command = commandTowardTarget(state, current, branch_path[1])) {
            return *command;
        }
    }

    const std::vector<GridIndex> path = pathToBestCoverageTarget(current);
    if (path.size() > 1U) {
        pending_target_ = path[1];
        if (auto command = commandTowardTarget(state, current, path[1])) {
            return *command;
        }
    }

    return finishedCommand();
}

} // namespace drone_mapper

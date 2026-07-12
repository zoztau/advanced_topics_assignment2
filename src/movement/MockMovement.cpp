#include <drone_mapper/MockMovement.h>

#include <drone_mapper/IMap3D.h>

#include <geometry/VoxelGeometry.h>

#include <mp-units/systems/si/math.h>

#include <algorithm>
#include <cmath>
#include <string>

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

[[nodiscard]] types::MovementResult blockedResult(std::string message) {
    return types::MovementResult{false, std::move(message)};
}

[[nodiscard]] bool blocksMovement(types::VoxelOccupancy occupancy) {
    return occupancy != types::VoxelOccupancy::Empty;
}

[[nodiscard]] int dimensionFromSpan(double min_value, double max_value, double resolution) {
    if (resolution <= 0.0 || max_value <= min_value) {
        return 0;
    }
    return static_cast<int>(std::ceil((max_value - min_value) / resolution));
}

[[nodiscard]] AxisIndexRange candidateRange(double from,
                                            double to,
                                            double radius,
                                            double map_min,
                                            double map_max,
                                            double resolution) {
    const int dimension = dimensionFromSpan(map_min, map_max, resolution);
    if (dimension == 0) {
        return {};
    }

    const double extent_min = std::min(from, to) - radius;
    const double extent_max = std::max(from, to) + radius;
    const int first = static_cast<int>(std::floor((extent_min - map_min) / resolution)) - 1;
    const int last = static_cast<int>(std::floor((extent_max - map_min) / resolution)) + 1;
    return AxisIndexRange{
        std::max(0, first),
        std::min(dimension - 1, last),
    };
}

[[nodiscard]] bool sphereFitsMapBounds(const Position3D& center,
                                       double radius,
                                       const types::MappingBounds& bounds) {
    return xCm(center.x) - radius >= xCm(bounds.min_x) &&
           xCm(center.x) + radius <= xCm(bounds.max_x) &&
           yCm(center.y) - radius >= yCm(bounds.min_y) &&
           yCm(center.y) + radius <= yCm(bounds.max_y) &&
           zCm(center.z) - radius >= zCm(bounds.min_height) &&
           zCm(center.z) + radius <= zCm(bounds.max_height);
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

} // namespace

MockMovement::MockMovement(MockGPS& gps) : gps_(gps) {}

MockMovement::MockMovement(MockGPS& gps, const IMap3D& hidden_map, PhysicalLength drone_radius)
    : gps_(gps),
      hidden_map_(&hidden_map),
      drone_radius_(drone_radius) {}

types::MovementResult MockMovement::rotate(types::RotationDirection direction, HorizontalAngle angle) {
    const Orientation current = gps_.heading();
    const HorizontalAngle signed_angle =
        (direction == types::RotationDirection::Left) ? angle : -angle;
    gps_.setHeading(Orientation{current.horizontal + signed_angle, current.altitude});
    return types::MovementResult{true, {}};
}

types::MovementResult MockMovement::advance(PhysicalLength distance) {
    const Position3D current = gps_.position();
    const Position3D target = advancedPosition(current, gps_.heading(), distance);
    if (!pathIsClear(current, target)) {
        return blockedResult("Advance command would collide with the hidden map.");
    }

    gps_.setPosition(target);
    return types::MovementResult{true, {}};
}

types::MovementResult MockMovement::elevate(PhysicalLength distance) {
    const Position3D current = gps_.position();
    const Position3D target = elevatedPosition(current, distance);
    if (!pathIsClear(current, target)) {
        return blockedResult("Elevate command would collide with the hidden map.");
    }

    gps_.setPosition(target);
    return types::MovementResult{true, {}};
}

bool MockMovement::pathIsClear(const Position3D& from, const Position3D& to) const {
    if (hidden_map_ == nullptr) {
        return true;
    }
    if (!sphereIsClear(from) || !sphereIsClear(to)) {
        return false;
    }

    const types::MapConfig config = hidden_map_->getMapConfig();
    const double resolution = lengthCm(config.resolution);
    if (resolution <= 0.0) {
        return false;
    }

    const double radius = std::max(0.0, lengthCm(drone_radius_));
    const AxisIndexRange x_range = candidateRange(
        xCm(from.x), xCm(to.x), radius,
        xCm(config.boundaries.min_x), xCm(config.boundaries.max_x), resolution);
    const AxisIndexRange y_range = candidateRange(
        yCm(from.y), yCm(to.y), radius,
        yCm(config.boundaries.min_y), yCm(config.boundaries.max_y), resolution);
    const AxisIndexRange z_range = candidateRange(
        zCm(from.z), zCm(to.z), radius,
        zCm(config.boundaries.min_height), zCm(config.boundaries.max_height), resolution);
    if (x_range.empty() || y_range.empty() || z_range.empty()) {
        return false;
    }

    for (int x = x_range.first; x <= x_range.last; ++x) {
        for (int y = y_range.first; y <= y_range.last; ++y) {
            for (int z = z_range.first; z <= z_range.last; ++z) {
                const geometry::VoxelAabb box = geometry::voxelAabb(config, x, y, z);
                if (!blocksMovement(hidden_map_->atVoxel(voxelProbe(box, config.boundaries)))) {
                    continue;
                }
                if (geometry::squaredDistanceFromSegmentToAabb(from, to, box) <=
                    radius * radius + kCollisionEpsilon) {
                    return false;
                }
            }
        }
    }

    return true;
}

bool MockMovement::sphereIsClear(const Position3D& center) const {
    if (hidden_map_ == nullptr) {
        return true;
    }

    const types::MapConfig config = hidden_map_->getMapConfig();
    const double resolution = lengthCm(config.resolution);
    const double radius = std::max(0.0, lengthCm(drone_radius_));
    if (resolution <= 0.0 || !hidden_map_->isInBounds(center) ||
        !sphereFitsMapBounds(center, radius, config.boundaries)) {
        return false;
    }

    const AxisIndexRange x_range = candidateRange(
        xCm(center.x), xCm(center.x), radius,
        xCm(config.boundaries.min_x), xCm(config.boundaries.max_x), resolution);
    const AxisIndexRange y_range = candidateRange(
        yCm(center.y), yCm(center.y), radius,
        yCm(config.boundaries.min_y), yCm(config.boundaries.max_y), resolution);
    const AxisIndexRange z_range = candidateRange(
        zCm(center.z), zCm(center.z), radius,
        zCm(config.boundaries.min_height), zCm(config.boundaries.max_height), resolution);
    if (x_range.empty() || y_range.empty() || z_range.empty()) {
        return false;
    }

    for (int x = x_range.first; x <= x_range.last; ++x) {
        for (int y = y_range.first; y <= y_range.last; ++y) {
            for (int z = z_range.first; z <= z_range.last; ++z) {
                const geometry::VoxelAabb box = geometry::voxelAabb(config, x, y, z);
                if (blocksMovement(hidden_map_->atVoxel(voxelProbe(box, config.boundaries))) &&
                    geometry::squaredDistanceToAabb(geometry::coordinatesCm(center), box) <=
                        radius * radius + kCollisionEpsilon) {
                    return false;
                }
            }
        }
    }

    return true;
}

} // namespace drone_mapper

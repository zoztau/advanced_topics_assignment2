#include <drone_mapper/MockMovement.h>

#include <drone_mapper/IMap3D.h>

#include <mp-units/systems/si/math.h>

#include <algorithm>
#include <cmath>
#include <string>

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

[[nodiscard]] types::MovementResult blockedResult(std::string message) {
    return types::MovementResult{false, std::move(message)};
}

[[nodiscard]] bool blocksMovement(types::VoxelOccupancy occupancy) {
    return occupancy != types::VoxelOccupancy::Empty;
}

[[nodiscard]] Position3D interpolatedPosition(const Position3D& from,
                                              const Position3D& to,
                                              double t) {
    return Position3D{
        (xCm(from.x) + (xCm(to.x) - xCm(from.x)) * t) * x_extent[cm],
        (yCm(from.y) + (yCm(to.y) - yCm(from.y)) * t) * y_extent[cm],
        (zCm(from.z) + (zCm(to.z) - zCm(from.z)) * t) * z_extent[cm],
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

    const double dx = xCm(to.x) - xCm(from.x);
    const double dy = yCm(to.y) - yCm(from.y);
    const double dz = zCm(to.z) - zCm(from.z);
    const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    const double resolution = lengthCm(hidden_map_->getMapConfig().resolution);
    const int samples = std::max(1, static_cast<int>(std::ceil(distance / std::max(resolution * 0.5, 1.0))));

    for (int sample = 0; sample <= samples; ++sample) {
        const double t = static_cast<double>(sample) / static_cast<double>(samples);
        if (!sphereIsClear(interpolatedPosition(from, to, t))) {
            return false;
        }
    }

    return true;
}

bool MockMovement::sphereIsClear(const Position3D& center) const {
    if (hidden_map_ == nullptr) {
        return true;
    }
    if (!hidden_map_->isInBounds(center) || blocksMovement(hidden_map_->atVoxel(center))) {
        return false;
    }

    const double radius = lengthCm(drone_radius_);
    const double resolution = lengthCm(hidden_map_->getMapConfig().resolution);
    if (radius <= 0.0 || resolution <= 0.0) {
        return true;
    }

    const int steps = std::max(1, static_cast<int>(std::ceil(radius / resolution)));
    for (int dx = -steps; dx <= steps; ++dx) {
        for (int dy = -steps; dy <= steps; ++dy) {
            for (int dz = -steps; dz <= steps; ++dz) {
                const double x_offset = static_cast<double>(dx) * resolution;
                const double y_offset = static_cast<double>(dy) * resolution;
                const double z_offset = static_cast<double>(dz) * resolution;
                if (x_offset * x_offset + y_offset * y_offset + z_offset * z_offset >
                    radius * radius + 1.0e-6) {
                    continue;
                }

                const Position3D sample{
                    (xCm(center.x) + x_offset) * x_extent[cm],
                    (yCm(center.y) + y_offset) * y_extent[cm],
                    (zCm(center.z) + z_offset) * z_extent[cm],
                };
                if (!hidden_map_->isInBounds(sample) || blocksMovement(hidden_map_->atVoxel(sample))) {
                    return false;
                }
            }
        }
    }

    return true;
}

} // namespace drone_mapper

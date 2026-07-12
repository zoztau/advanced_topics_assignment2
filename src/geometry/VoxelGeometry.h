#pragma once

#include <drone_mapper/Types.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace drone_mapper::geometry {

struct VoxelAabb {
    std::array<double, 3> min{};
    std::array<double, 3> max{};
};

[[nodiscard]] inline std::array<double, 3> coordinatesCm(const Position3D& position) {
    return {
        position.x.force_numerical_value_in(cm),
        position.y.force_numerical_value_in(cm),
        position.z.force_numerical_value_in(cm),
    };
}

[[nodiscard]] inline VoxelAabb voxelAabb(const types::MapConfig& config,
                                         int x_index,
                                         int y_index,
                                         int z_index) {
    const double resolution = config.resolution.force_numerical_value_in(cm);
    const double x_min = config.boundaries.min_x.force_numerical_value_in(cm) +
                         static_cast<double>(x_index) * resolution;
    const double y_min = config.boundaries.min_y.force_numerical_value_in(cm) +
                         static_cast<double>(y_index) * resolution;
    const double z_min = config.boundaries.min_height.force_numerical_value_in(cm) +
                         static_cast<double>(z_index) * resolution;
    return VoxelAabb{
        {x_min, y_min, z_min},
        {x_min + resolution, y_min + resolution, z_min + resolution},
    };
}

[[nodiscard]] inline double squaredDistanceToAabb(const std::array<double, 3>& point,
                                                  const VoxelAabb& box) {
    double distance_squared = 0.0;
    for (std::size_t axis = 0; axis < point.size(); ++axis) {
        const double distance = point[axis] < box.min[axis]
                                    ? box.min[axis] - point[axis]
                                    : (point[axis] > box.max[axis]
                                           ? point[axis] - box.max[axis]
                                           : 0.0);
        distance_squared += distance * distance;
    }
    return distance_squared;
}

[[nodiscard]] inline double squaredDistanceToVoxelAabb(const Position3D& position,
                                                       int x_index,
                                                       int y_index,
                                                       int z_index,
                                                       const types::MapConfig& config) {
    return squaredDistanceToAabb(coordinatesCm(position),
                                 voxelAabb(config, x_index, y_index, z_index));
}

[[nodiscard]] inline double squaredDistanceFromSegmentToAabb(const Position3D& from,
                                                             const Position3D& to,
                                                             const VoxelAabb& box) {
    const std::array<double, 3> start = coordinatesCm(from);
    const std::array<double, 3> finish = coordinatesCm(to);
    std::array<double, 3> delta{};
    for (std::size_t axis = 0; axis < delta.size(); ++axis) {
        delta[axis] = finish[axis] - start[axis];
    }

    std::array<double, 8> breakpoints{};
    std::size_t breakpoint_count = 2;
    breakpoints[0] = 0.0;
    breakpoints[1] = 1.0;
    for (std::size_t axis = 0; axis < delta.size(); ++axis) {
        if (delta[axis] == 0.0) {
            continue;
        }
        for (const double boundary : {box.min[axis], box.max[axis]}) {
            const double t = (boundary - start[axis]) / delta[axis];
            if (t > 0.0 && t < 1.0) {
                breakpoints[breakpoint_count++] = t;
            }
        }
    }
    std::sort(breakpoints.begin(), breakpoints.begin() + static_cast<std::ptrdiff_t>(breakpoint_count));

    const auto pointAt = [&](double t) {
        std::array<double, 3> point{};
        for (std::size_t axis = 0; axis < point.size(); ++axis) {
            point[axis] = start[axis] + delta[axis] * t;
        }
        return point;
    };

    double minimum = squaredDistanceToAabb(start, box);
    for (std::size_t i = 0; i < breakpoint_count; ++i) {
        minimum = std::min(minimum, squaredDistanceToAabb(pointAt(breakpoints[i]), box));
    }

    for (std::size_t i = 0; i + 1 < breakpoint_count; ++i) {
        const double interval_start = breakpoints[i];
        const double interval_end = breakpoints[i + 1];
        if (interval_start == interval_end) {
            continue;
        }

        const double midpoint = (interval_start + interval_end) * 0.5;
        double linear_product_sum = 0.0;
        double delta_squared_sum = 0.0;
        for (std::size_t axis = 0; axis < delta.size(); ++axis) {
            const double midpoint_value = start[axis] + delta[axis] * midpoint;
            double boundary = 0.0;
            if (midpoint_value < box.min[axis]) {
                boundary = box.min[axis];
            } else if (midpoint_value > box.max[axis]) {
                boundary = box.max[axis];
            } else {
                continue;
            }

            linear_product_sum += (start[axis] - boundary) * delta[axis];
            delta_squared_sum += delta[axis] * delta[axis];
        }

        if (delta_squared_sum > 0.0) {
            const double stationary = std::clamp(-linear_product_sum / delta_squared_sum,
                                                 interval_start,
                                                 interval_end);
            minimum = std::min(minimum, squaredDistanceToAabb(pointAt(stationary), box));
        }
    }

    return minimum;
}

} // namespace drone_mapper::geometry

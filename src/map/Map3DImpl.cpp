#include <drone_mapper/Map3DImpl.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <stdexcept>
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

[[nodiscard]] std::size_t checkedCellCount(const std::array<std::size_t, 3>& dimensions) {
    if (dimensions[0] == 0 || dimensions[1] == 0 || dimensions[2] == 0) {
        throw std::runtime_error("Map dimensions must be positive.");
    }
    return dimensions[0] * dimensions[1] * dimensions[2];
}

[[nodiscard]] bool hasUsableResolution(const types::MapConfig& config) {
    return lengthCm(config.resolution) > 0.0;
}

[[nodiscard]] bool hasUsableBounds(const types::MapConfig& config) {
    return xCm(config.boundaries.max_x) > xCm(config.boundaries.min_x) &&
           yCm(config.boundaries.max_y) > yCm(config.boundaries.min_y) &&
           zCm(config.boundaries.max_height) > zCm(config.boundaries.min_height);
}

[[nodiscard]] std::size_t dimensionFromSpan(double min_value, double max_value, double resolution) {
    if (resolution <= 0.0 || max_value <= min_value) {
        return 1;
    }
    return static_cast<std::size_t>(std::ceil((max_value - min_value) / resolution));
}

[[nodiscard]] std::array<std::size_t, 3> dimensionsFromConfig(const types::MapConfig& config) {
    if (!hasUsableResolution(config) || !hasUsableBounds(config)) {
        return {1, 1, 1};
    }

    const double resolution = lengthCm(config.resolution);
    return {
        dimensionFromSpan(xCm(config.boundaries.min_x), xCm(config.boundaries.max_x), resolution),
        dimensionFromSpan(yCm(config.boundaries.min_y), yCm(config.boundaries.max_y), resolution),
        dimensionFromSpan(zCm(config.boundaries.min_height), zCm(config.boundaries.max_height), resolution),
    };
}

[[nodiscard]] types::MapConfig normalizedConfig(types::MapConfig config,
                                                const std::array<std::size_t, 3>& dimensions) {
    if (!hasUsableResolution(config)) {
        config.resolution = 1.0 * cm;
    }

    if (!hasUsableBounds(config)) {
        const double resolution = lengthCm(config.resolution);
        const double x_offset = xCm(config.offset.x);
        const double y_offset = yCm(config.offset.y);
        const double z_offset = zCm(config.offset.z);
        config.boundaries = types::MappingBounds{
            (-x_offset) * x_extent[cm],
            (static_cast<double>(dimensions[0]) * resolution - x_offset) * x_extent[cm],
            (-y_offset) * y_extent[cm],
            (static_cast<double>(dimensions[1]) * resolution - y_offset) * y_extent[cm],
            (-z_offset) * z_extent[cm],
            (static_cast<double>(dimensions[2]) * resolution - z_offset) * z_extent[cm],
        };
    }

    return config;
}

[[nodiscard]] std::uint32_t readLittleEndianU32(std::ifstream& input, std::size_t byte_count) {
    std::uint32_t value = 0;
    for (std::size_t i = 0; i < byte_count; ++i) {
        const int byte = input.get();
        if (byte == std::char_traits<char>::eof()) {
            throw std::runtime_error("Unexpected end of npy header.");
        }
        value |= static_cast<std::uint32_t>(static_cast<unsigned char>(byte)) << (8U * i);
    }
    return value;
}

[[nodiscard]] std::string headerValue(const std::string& header, const std::string& key) {
    const std::string marker = "'" + key + "'";
    const std::size_t key_pos = header.find(marker);
    if (key_pos == std::string::npos) {
        throw std::runtime_error("Npy header is missing key: " + key);
    }

    const std::size_t colon_pos = header.find(':', key_pos);
    const std::size_t comma_pos = header.find(',', colon_pos);
    if (colon_pos == std::string::npos || comma_pos == std::string::npos) {
        throw std::runtime_error("Malformed npy header value for key: " + key);
    }

    return header.substr(colon_pos + 1, comma_pos - colon_pos - 1);
}

[[nodiscard]] std::string parseDescr(const std::string& header) {
    const std::string value = headerValue(header, "descr");
    const std::size_t first_quote = value.find('\'');
    const std::size_t last_quote = value.rfind('\'');
    if (first_quote == std::string::npos || last_quote == first_quote) {
        throw std::runtime_error("Malformed npy dtype descriptor.");
    }
    return value.substr(first_quote + 1, last_quote - first_quote - 1);
}

[[nodiscard]] std::array<std::size_t, 3> parseShape(const std::string& header) {
    const std::size_t shape_pos = header.find("'shape'");
    const std::size_t open_pos = header.find('(', shape_pos);
    const std::size_t close_pos = header.find(')', open_pos);
    if (shape_pos == std::string::npos || open_pos == std::string::npos || close_pos == std::string::npos) {
        throw std::runtime_error("Npy header must contain a 3D shape.");
    }

    std::string shape_text = header.substr(open_pos + 1, close_pos - open_pos - 1);
    std::replace(shape_text.begin(), shape_text.end(), ',', ' ');
    std::istringstream stream(shape_text);

    std::array<std::size_t, 3> dimensions{};
    stream >> dimensions[0] >> dimensions[1] >> dimensions[2];
    if (!stream || dimensions[0] == 0 || dimensions[1] == 0 || dimensions[2] == 0) {
        throw std::runtime_error("Npy map shape must be three positive dimensions.");
    }

    return dimensions;
}

[[nodiscard]] types::VoxelOccupancy occupancyFromInt(int value) {
    switch (value) {
    case -3:
        return types::VoxelOccupancy::PotentiallyOccupied;
    case -2:
        return types::VoxelOccupancy::OutOfBounds;
    case -1:
        return types::VoxelOccupancy::Unmapped;
    case 0:
        return types::VoxelOccupancy::Empty;
    case 1:
        return types::VoxelOccupancy::Occupied;
    default:
        return value > 0 ? types::VoxelOccupancy::Occupied : types::VoxelOccupancy::Unmapped;
    }
}

[[nodiscard]] int toInt(types::VoxelOccupancy occupancy) {
    return static_cast<int>(occupancy);
}

[[nodiscard]] std::string makeNpyHeader(const std::array<std::size_t, 3>& dimensions) {
    std::ostringstream stream;
    stream << "{'descr': '|i1', 'fortran_order': False, 'shape': (" << dimensions[0] << ", "
           << dimensions[1] << ", " << dimensions[2] << "), }";

    std::string header = stream.str();
    constexpr std::size_t header_prefix_size = 10;
    const std::size_t padding = 16 - ((header_prefix_size + header.size() + 1) % 16);
    header.append(padding, ' ');
    header.push_back('\n');
    return header;
}

} // namespace

Map3DImpl::Map3DImpl(std::shared_ptr<NpyArray> map_ptr)
    : Map3DImpl(std::move(map_ptr), types::MapConfig{}, {1, 1, 1}, {types::VoxelOccupancy::Unmapped}) {}

Map3DImpl::Map3DImpl(std::shared_ptr<NpyArray> map_ptr, const types::MapConfig map_config)
    : Map3DImpl(std::move(map_ptr),
                normalizedConfig(map_config, dimensionsFromConfig(map_config)),
                dimensionsFromConfig(map_config),
                std::vector<types::VoxelOccupancy>(
                    checkedCellCount(dimensionsFromConfig(map_config)),
                    types::VoxelOccupancy::Unmapped)) {}

Map3DImpl::Map3DImpl(std::shared_ptr<NpyArray> map_ptr,
                     types::MapConfig map_config,
                     std::array<std::size_t, 3> dimensions,
                     std::vector<types::VoxelOccupancy> cells)
    : map_(std::move(map_ptr)),
      config_(normalizedConfig(map_config, dimensions)),
      dimensions_(dimensions),
      cells_(std::move(cells)) {
    if (!map_) {
        throw std::invalid_argument("Map3DImpl requires a valid map pointer.");
    }
    if (cells_.size() != checkedCellCount(dimensions_)) {
        throw std::invalid_argument("Map3DImpl cell buffer size does not match dimensions.");
    }
}

std::unique_ptr<Map3DImpl> Map3DImpl::loadFromNpyFile(const std::filesystem::path& path,
                                                      PhysicalLength resolution,
                                                      Position3D offset) {
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        throw std::runtime_error("Could not open npy map file: " + path.string());
    }

    std::array<char, 6> magic{};
    input.read(magic.data(), static_cast<std::streamsize>(magic.size()));
    if (!input || std::string(magic.data(), magic.size()) != "\x93NUMPY") {
        throw std::runtime_error("Map file is not a valid npy file: " + path.string());
    }

    const int major_version = input.get();
    const int minor_version = input.get();
    (void)minor_version;
    if (major_version != 1 && major_version != 2) {
        throw std::runtime_error("Unsupported npy version in map file: " + path.string());
    }

    const std::uint32_t header_size = readLittleEndianU32(input, major_version == 1 ? 2U : 4U);
    std::string header(header_size, '\0');
    input.read(header.data(), static_cast<std::streamsize>(header.size()));
    if (!input) {
        throw std::runtime_error("Could not read npy map header: " + path.string());
    }

    const std::string dtype = parseDescr(header);
    const bool unsigned_byte = dtype == "|u1" || dtype == "u1";
    const bool signed_byte = dtype == "|i1" || dtype == "i1";
    if (!unsigned_byte && !signed_byte) {
        throw std::runtime_error("Only 8-bit npy map arrays are supported: " + path.string());
    }
    if (headerValue(header, "fortran_order").find("False") == std::string::npos) {
        throw std::runtime_error("Fortran-order npy maps are not supported: " + path.string());
    }

    const std::array<std::size_t, 3> dimensions = parseShape(header);
    std::vector<types::VoxelOccupancy> cells;
    cells.reserve(checkedCellCount(dimensions));
    for (std::size_t i = 0; i < checkedCellCount(dimensions); ++i) {
        const int byte = input.get();
        if (byte == std::char_traits<char>::eof()) {
            throw std::runtime_error("Npy map data ended before all cells were read: " + path.string());
        }

        const int value = signed_byte ? static_cast<int>(static_cast<std::int8_t>(byte))
                                      : static_cast<int>(static_cast<std::uint8_t>(byte));
        cells.push_back(occupancyFromInt(value));
    }

    types::MapConfig config{};
    config.resolution = resolution;
    config.offset = offset;
    return std::unique_ptr<Map3DImpl>{
        new Map3DImpl(std::make_shared<NpyArray>(), config, dimensions, std::move(cells))};
}

std::unique_ptr<Map3DImpl> Map3DImpl::createEmpty(const types::MapConfig& map_config,
                                                  types::VoxelOccupancy initial_value) {
    const std::array<std::size_t, 3> dimensions = dimensionsFromConfig(map_config);
    return std::unique_ptr<Map3DImpl>{
        new Map3DImpl(std::make_shared<NpyArray>(),
                      map_config,
                      dimensions,
                      std::vector<types::VoxelOccupancy>(checkedCellCount(dimensions), initial_value))};
}

types::VoxelOccupancy Map3DImpl::atVoxel(const Position3D& pos) const {
    if (!isInBounds(pos)) {
        return types::VoxelOccupancy::OutOfBounds;
    }
    return cells_.at(flatIndex(worldToGrid(pos)));
}

types::MapConfig Map3DImpl::getMapConfig() const {
    return config_;
}

bool Map3DImpl::isInBounds(const Position3D& pos) const {
    if (!hasUsableBounds(config_) || !hasUsableResolution(config_)) {
        return false;
    }

    if (xCm(pos.x) < xCm(config_.boundaries.min_x) ||
        xCm(pos.x) >= xCm(config_.boundaries.max_x) ||
        yCm(pos.y) < yCm(config_.boundaries.min_y) ||
        yCm(pos.y) >= yCm(config_.boundaries.max_y) ||
        zCm(pos.z) < zCm(config_.boundaries.min_height) ||
        zCm(pos.z) >= zCm(config_.boundaries.max_height)) {
        return false;
    }

    return containsIndex(worldToGrid(pos));
}

std::array<std::size_t, 3> Map3DImpl::dimensions() const {
    return dimensions_;
}

void Map3DImpl::set(const Position3D& pos, types::VoxelOccupancy value) {
    if (!isInBounds(pos)) {
        return;
    }
    cells_.at(flatIndex(worldToGrid(pos))) = value;
}

void Map3DImpl::save(const std::filesystem::path& path) const {
    if (path.has_parent_path()) {
        std::filesystem::create_directories(path.parent_path());
    }

    std::ofstream output{path, std::ios::binary};
    if (!output) {
        throw std::runtime_error("Failed to create output map file: " + path.string());
    }

    const std::string header = makeNpyHeader(dimensions_);
    output.write("\x93NUMPY", 6);
    output.put(static_cast<char>(1));
    output.put(static_cast<char>(0));
    const auto header_size = static_cast<std::uint16_t>(header.size());
    output.put(static_cast<char>(header_size & 0xFFU));
    output.put(static_cast<char>((header_size >> 8U) & 0xFFU));
    output.write(header.data(), static_cast<std::streamsize>(header.size()));

    for (types::VoxelOccupancy cell : cells_) {
        output.put(static_cast<char>(static_cast<std::int8_t>(toInt(cell))));
    }
}

bool Map3DImpl::containsIndex(const GridIndex& index) const noexcept {
    if (index.x < 0 || index.y < 0 || index.z < 0) {
        return false;
    }
    return static_cast<std::size_t>(index.x) < dimensions_[0] &&
           static_cast<std::size_t>(index.y) < dimensions_[1] &&
           static_cast<std::size_t>(index.z) < dimensions_[2];
}

std::size_t Map3DImpl::flatIndex(const GridIndex& index) const {
    return static_cast<std::size_t>(index.x) * dimensions_[1] * dimensions_[2] +
           static_cast<std::size_t>(index.y) * dimensions_[2] +
           static_cast<std::size_t>(index.z);
}

Map3DImpl::GridIndex Map3DImpl::worldToGrid(const Position3D& pos) const {
    const double resolution = lengthCm(config_.resolution);
    return GridIndex{
        static_cast<int>(std::floor((xCm(pos.x) + xCm(config_.offset.x)) / resolution)),
        static_cast<int>(std::floor((yCm(pos.y) + yCm(config_.offset.y)) / resolution)),
        static_cast<int>(std::floor((zCm(pos.z) + zCm(config_.offset.z)) / resolution)),
    };
}

} // namespace drone_mapper

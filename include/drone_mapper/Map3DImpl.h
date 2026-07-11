#pragma once

#include <TinyNPY.h>

#include <drone_mapper/IMutableMap3D.h>

#include <array>
#include <filesystem>
#include <memory>
#include <vector>

namespace drone_mapper {

class Map3DImpl final : public IMutableMap3D {
public:
    Map3DImpl(std::shared_ptr<NpyArray> map_ptr);
    // Changed: added offset-aware construction for hidden maps loaded from NPY files.
    Map3DImpl(std::shared_ptr<NpyArray> map_ptr, const types::MapConfig map_config);

    [[nodiscard]] static std::unique_ptr<Map3DImpl> loadFromNpyFile(
        const std::filesystem::path& path,
        PhysicalLength resolution,
        Position3D offset);
    [[nodiscard]] static std::unique_ptr<Map3DImpl> createEmpty(
        const types::MapConfig& map_config,
        types::VoxelOccupancy initial_value = types::VoxelOccupancy::Unmapped);

    [[nodiscard]] types::VoxelOccupancy atVoxel(const Position3D& pos) const override;
    // Changed: exposes boundaries, offset, and resolution as one map-owned configuration.
    [[nodiscard]] types::MapConfig getMapConfig() const override;
    [[nodiscard]] bool isInBounds(const Position3D& pos) const override;
    [[nodiscard]] std::array<std::size_t, 3> dimensions() const;

    //Mutable map methods
    void set(const Position3D& pos, types::VoxelOccupancy value) override;
    void save(const std::filesystem::path& output_path) const override;

private:
    struct GridIndex {
        int x = 0;
        int y = 0;
        int z = 0;
    };

    Map3DImpl(std::shared_ptr<NpyArray> map_ptr,
              types::MapConfig map_config,
              std::array<std::size_t, 3> dimensions,
              std::vector<types::VoxelOccupancy> cells);

    [[nodiscard]] bool containsIndex(const GridIndex& index) const noexcept;
    [[nodiscard]] std::size_t flatIndex(const GridIndex& index) const;
    [[nodiscard]] GridIndex worldToGrid(const Position3D& pos) const;

    // Changed: shared ownership supports the new pointer-based storage member.
    std::shared_ptr<NpyArray> map_;
    // Changed: replaces standalone resolution_ so all map geometry stays together.
    types::MapConfig config_;
    std::array<std::size_t, 3> dimensions_{1, 1, 1};
    std::vector<types::VoxelOccupancy> cells_{types::VoxelOccupancy::Unmapped};
};

} // namespace drone_mapper

#include <drone_mapper/DroneControlImpl.h>
#include <drone_mapper/IMappingAlgorithm.h>
#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MissionControlImpl.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockLidar.h>
#include <drone_mapper/MockMovement.h>
#include <drone_mapper/SimulationManager.h>
#include <drone_mapper/SimulationRunFactory.h>
#include <drone_mapper/SimulationRunImpl.h>

#include <gtest/gtest.h>

#include <filesystem>
#include <memory>
#include <optional>
#include <vector>

namespace drone_mapper {
namespace {

[[nodiscard]] Position3D pos(double x, double y, double z) {
    return Position3D{x * x_extent[cm], y * y_extent[cm], z * z_extent[cm]};
}

[[nodiscard]] Orientation makeHeading(double horizontal, double altitude = 0.0) {
    return Orientation{horizontal * horizontal_angle[deg], altitude * altitude_angle[deg]};
}

[[nodiscard]] types::MappingBounds bounds(double min_x,
                                          double max_x,
                                          double min_y,
                                          double max_y,
                                          double min_z,
                                          double max_z) {
    return types::MappingBounds{
        min_x * x_extent[cm],
        max_x * x_extent[cm],
        min_y * y_extent[cm],
        max_y * y_extent[cm],
        min_z * z_extent[cm],
        max_z * z_extent[cm],
    };
}

[[nodiscard]] types::MapConfig mapConfig(types::MappingBounds map_bounds, double resolution) {
    return types::MapConfig{
        map_bounds,
        Position3D{
            -map_bounds.min_x.force_numerical_value_in(cm) * x_extent[cm],
            -map_bounds.min_y.force_numerical_value_in(cm) * y_extent[cm],
            -map_bounds.min_height.force_numerical_value_in(cm) * z_extent[cm],
        },
        resolution * cm,
    };
}

[[nodiscard]] types::MissionConfigData missionConfig() {
    return types::MissionConfigData{
        20,
        10.0 * cm,
        1.0,
        bounds(0.0, 50.0, 0.0, 50.0, 0.0, 50.0),
    };
}

[[nodiscard]] types::DroneConfigData droneConfig() {
    return types::DroneConfigData{
        1.0 * cm,
        90.0 * horizontal_angle[deg],
        20.0 * cm,
        20.0 * cm,
    };
}

[[nodiscard]] types::LidarConfigData lidarConfig() {
    return types::LidarConfigData{
        5.0 * cm,
        30.0 * cm,
        2.0 * cm,
        1,
    };
}

class OneScanAlgorithm final : public IMappingAlgorithm {
public:
    OneScanAlgorithm(const types::MissionConfigData& mission,
                     const types::LidarConfigData& lidar,
                     const types::DroneConfigData& drone,
                     const IMap3D& output_map)
        : IMappingAlgorithm(mission, lidar, drone, output_map) {}

    types::MappingStepCommand nextStep(const types::DroneState&, const types::LidarScanResult*) override {
        if (called_) {
            return types::MappingStepCommand{std::nullopt, std::nullopt, types::AlgorithmStatus::Finished};
        }
        called_ = true;
        return types::MappingStepCommand{std::nullopt, makeHeading(0.0), types::AlgorithmStatus::Working};
    }

private:
    bool called_ = false;
};

} // namespace

TEST(Integration, FullFlowWithRealAlgorithmProducesReportAndOutputFolder) {
    types::SimulationCompositionData composition{};
    composition.simulation_mission_groups.emplace_back(
        types::SimulationConfigData{
            std::filesystem::current_path() / "data_maps/single_voxel_x2_y4_z2.npy",
            10.0 * cm,
            Position3D{},
            pos(10.0, 10.0, 10.0),
            0.0 * horizontal_angle[deg],
        },
        std::vector{missionConfig()});
    composition.drones = {droneConfig()};
    composition.lidars = {lidarConfig()};

    SimulationManager manager{std::make_unique<SimulationRunFactory>()};
    const std::filesystem::path output_path =
        std::filesystem::temp_directory_path() / "drone_mapper_real_integration_test";

    const types::SimulationManagerReport report = manager.run(composition, output_path);

    ASSERT_EQ(report.runs.size(), 1U);
    EXPECT_TRUE(std::filesystem::exists(output_path / "simulation_output.yaml"));
    EXPECT_TRUE(std::filesystem::exists(output_path / "output_results"));
    EXPECT_GE(report.runs.front().mission_score, 0.0);
}

TEST(Integration, FullFlowWithMockAlgorithmCompletesSingleRun) {
    const types::MapConfig config = mapConfig(bounds(0.0, 50.0, 0.0, 50.0, 0.0, 50.0), 10.0);
    auto hidden = Map3DImpl::createEmpty(config, types::VoxelOccupancy::Empty);
    auto output = Map3DImpl::createEmpty(config, types::VoxelOccupancy::Unmapped);
    auto gps = std::make_unique<MockGPS>(pos(10.0, 10.0, 10.0), makeHeading(0.0), 10.0 * cm);
    auto movement = std::make_unique<MockMovement>(*gps);
    auto lidar = std::make_unique<MockLidar>(lidarConfig(), *hidden, *gps);
    auto algorithm = std::make_unique<OneScanAlgorithm>(missionConfig(), lidarConfig(), droneConfig(), *output);
    auto drone_control = std::make_unique<DroneControlImpl>(
        droneConfig(),
        missionConfig(),
        *lidar,
        *gps,
        *movement,
        *output,
        *algorithm);
    const std::filesystem::path output_map =
        std::filesystem::temp_directory_path() / "drone_mapper_mock_algorithm_output.npy";
    auto mission_control = std::make_unique<MissionControlImpl>(
        missionConfig(),
        droneConfig(),
        *hidden,
        *output,
        *drone_control,
        output_map);

    SimulationRunImpl run{
        std::move(hidden),
        std::move(output),
        std::move(gps),
        std::move(movement),
        std::move(lidar),
        std::move(algorithm),
        std::move(drone_control),
        std::move(mission_control),
        types::SimulationConfigData{},
        missionConfig(),
        types::ResolutionRequestStatus::Accepted,
        output_map,
    };

    const types::SimulationResult result = run.run();

    ASSERT_EQ(result.mission_results.size(), 1U);
    EXPECT_EQ(result.mission_results.front().status, types::MissionRunStatus::Completed);
    EXPECT_GE(result.mission_score, 0.0);
}

} // namespace drone_mapper

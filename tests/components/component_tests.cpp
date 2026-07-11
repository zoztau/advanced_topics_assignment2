#include <drone_mapper/DroneControlImpl.h>
#include <drone_mapper/IDroneControl.h>
#include <drone_mapper/ILidar.h>
#include <drone_mapper/IMappingAlgorithm.h>
#include <drone_mapper/IMissionControl.h>
#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MappingAlgorithmImpl.h>
#include <drone_mapper/MapsComparison.h>
#include <drone_mapper/MissionControlImpl.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockLidar.h>
#include <drone_mapper/MockMovement.h>
#include <drone_mapper/SimulationManager.h>
#include <drone_mapper/SimulationRunImpl.h>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
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
        10,
        10.0 * cm,
        1.0,
        bounds(0.0, 100.0, 0.0, 100.0, 0.0, 100.0),
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
        40.0 * cm,
        2.0 * cm,
        1,
    };
}

class StaticRun final : public ISimulationRun {
public:
    explicit StaticRun(double score) : score_(score) {}

    types::SimulationResult run() override {
        types::SimulationResult result{};
        result.mission_score = score_;
        result.mission_results.push_back(types::MissionRunResult{types::MissionRunStatus::Completed, 1, {}});
        return result;
    }

private:
    double score_ = 0.0;
};

class CountingFactory final : public ISimulationRunFactory {
public:
    std::unique_ptr<ISimulationRun> create(const types::SimulationConfigData&,
                                           const types::MissionConfigData&,
                                           const types::DroneConfigData&,
                                           const types::LidarConfigData&,
                                           const std::filesystem::path&) override {
        ++create_count;
        return std::make_unique<StaticRun>(100.0);
    }

    int create_count = 0;
};

class SequenceDroneControl final : public IDroneControl {
public:
    explicit SequenceDroneControl(std::vector<types::DroneStepResult> results)
        : results_(std::move(results)) {}

    types::DroneStepResult step() override {
        if (next_ >= results_.size()) {
            return types::DroneStepResult{types::DroneStepStatus::Completed, {}};
        }
        return results_[next_++];
    }

    types::DroneState state() const override {
        return types::DroneState{pos(10.0, 10.0, 10.0), makeHeading(0.0), next_};
    }

private:
    std::vector<types::DroneStepResult> results_;
    std::size_t next_ = 0;
};

class ScriptedAlgorithm final : public IMappingAlgorithm {
public:
    ScriptedAlgorithm(const types::MissionConfigData& mission,
                      const types::LidarConfigData& lidar,
                      const types::DroneConfigData& drone,
                      const IMap3D& output_map,
                      types::MappingStepCommand first_command)
        : IMappingAlgorithm(mission, lidar, drone, output_map),
          first_command_(std::move(first_command)) {}

    types::MappingStepCommand nextStep(const types::DroneState&, const types::LidarScanResult* latest_scan) override {
        saw_null_latest_scan = latest_scan == nullptr;
        ++calls;
        if (calls == 1) {
            return first_command_;
        }
        return types::MappingStepCommand{std::nullopt, std::nullopt, types::AlgorithmStatus::Finished};
    }

    bool saw_null_latest_scan = false;
    int calls = 0;

private:
    types::MappingStepCommand first_command_;
};

class FixedLidar final : public ILidar {
public:
    explicit FixedLidar(types::LidarConfigData config) : config_(config) {}

    types::LidarScanResult scan(Orientation scan_orientation) const override {
        ++scan_calls;
        return {types::LidarHit{std::numeric_limits<double>::max() * cm, scan_orientation}};
    }

    types::LidarConfigData config() const override {
        return config_;
    }

    mutable int scan_calls = 0;

private:
    types::LidarConfigData config_;
};

class NullGPS final : public IGPS {
public:
    Position3D position() const override {
        return pos(10.0, 10.0, 10.0);
    }

    Orientation heading() const override {
        return makeHeading(0.0);
    }
};

class NullMovement final : public IDroneMovement {
public:
    types::MovementResult rotate(types::RotationDirection, HorizontalAngle) override {
        return types::MovementResult{true, {}};
    }

    types::MovementResult advance(PhysicalLength) override {
        return types::MovementResult{true, {}};
    }

    types::MovementResult elevate(PhysicalLength) override {
        return types::MovementResult{true, {}};
    }
};

class NullLidar final : public ILidar {
public:
    types::LidarScanResult scan(Orientation) const override {
        return {};
    }

    types::LidarConfigData config() const override {
        return lidarConfig();
    }
};

class NullAlgorithm final : public IMappingAlgorithm {
public:
    NullAlgorithm(const IMap3D& output_map)
        : IMappingAlgorithm(missionConfig(), lidarConfig(), droneConfig(), output_map) {}

    types::MappingStepCommand nextStep(const types::DroneState&, const types::LidarScanResult*) override {
        return types::MappingStepCommand{std::nullopt, std::nullopt, types::AlgorithmStatus::Finished};
    }
};

class NullDroneControl final : public IDroneControl {
public:
    types::DroneStepResult step() override {
        return types::DroneStepResult{types::DroneStepStatus::Completed, {}};
    }

    types::DroneState state() const override {
        return types::DroneState{pos(10.0, 10.0, 10.0), makeHeading(0.0), 0};
    }
};

class CompletedMission final : public IMissionControl {
public:
    types::MissionRunResult runMission() override {
        called = true;
        return types::MissionRunResult{types::MissionRunStatus::Completed, 1, {}};
    }

    bool called = false;
};

} // namespace

TEST(SimulationManager, ExpandsCartesianProductAndWritesReport) {
    auto factory = std::make_unique<CountingFactory>();
    CountingFactory* factory_ptr = factory.get();
    SimulationManager manager{std::move(factory)};

    types::SimulationCompositionData composition{};
    composition.simulation_mission_groups.emplace_back(
        types::SimulationConfigData{},
        std::vector{missionConfig(), missionConfig()});
    composition.drones = {droneConfig(), droneConfig()};
    composition.lidars = {lidarConfig(), lidarConfig()};

    const std::filesystem::path output_path =
        std::filesystem::temp_directory_path() / "drone_mapper_simulation_manager_test";
    const types::SimulationManagerReport report = manager.run(composition, output_path);

    EXPECT_EQ(factory_ptr->create_count, 8);
    EXPECT_EQ(report.runs.size(), 8U);
    EXPECT_TRUE(std::filesystem::exists(output_path / "simulation_output.yaml"));
}

TEST(SimulationRun, CallsMissionAndScoresCompletedMap) {
    const types::MapConfig config = mapConfig(bounds(0.0, 20.0, 0.0, 20.0, 0.0, 20.0), 10.0);
    auto hidden = Map3DImpl::createEmpty(config, types::VoxelOccupancy::Empty);
    auto output = Map3DImpl::createEmpty(config, types::VoxelOccupancy::Empty);
    auto output_for_algorithm = Map3DImpl::createEmpty(config, types::VoxelOccupancy::Empty);
    auto mission = std::make_unique<CompletedMission>();

    SimulationRunImpl run{
        std::move(hidden),
        std::move(output),
        std::make_unique<NullGPS>(),
        std::make_unique<NullMovement>(),
        std::make_unique<NullLidar>(),
        std::make_unique<NullAlgorithm>(*output_for_algorithm),
        std::make_unique<NullDroneControl>(),
        std::move(mission),
        types::SimulationConfigData{},
        missionConfig(),
        types::ResolutionRequestStatus::Accepted,
        std::filesystem::temp_directory_path() / "simulation_run_output.npy",
    };

    const types::SimulationResult result = run.run();

    ASSERT_EQ(result.mission_results.size(), 1U);
    EXPECT_EQ(result.mission_results.front().status, types::MissionRunStatus::Completed);
    EXPECT_DOUBLE_EQ(result.mission_score, 100.0);
}

TEST(MissionControl, StopsWhenDroneReportsCompleted) {
    const types::MapConfig config = mapConfig(bounds(0.0, 50.0, 0.0, 50.0, 0.0, 50.0), 10.0);
    auto hidden = Map3DImpl::createEmpty(config, types::VoxelOccupancy::Empty);
    auto output = Map3DImpl::createEmpty(config, types::VoxelOccupancy::Unmapped);
    SequenceDroneControl drone_control{
        {types::DroneStepResult{types::DroneStepStatus::Continue, {}},
         types::DroneStepResult{types::DroneStepStatus::Completed, {}}}};
    MissionControlImpl mission{
        missionConfig(),
        droneConfig(),
        *hidden,
        *output,
        drone_control,
        std::filesystem::temp_directory_path() / "mission_control_output.npy",
    };

    const types::MissionRunResult result = mission.runMission();

    EXPECT_EQ(result.status, types::MissionRunStatus::Completed);
    EXPECT_EQ(result.steps, 2U);
}

TEST(DroneControl, FirstStepPassesNullScanAndAppliesLidarMiss) {
    const types::MapConfig config = mapConfig(bounds(0.0, 80.0, 0.0, 80.0, 0.0, 80.0), 10.0);
    auto output = Map3DImpl::createEmpty(config, types::VoxelOccupancy::Unmapped);
    MockGPS gps{pos(10.0, 10.0, 10.0), makeHeading(0.0), 10.0 * cm};
    MockMovement movement{gps};
    FixedLidar lidar{lidarConfig()};
    types::MappingStepCommand first_command{
        std::nullopt,
        makeHeading(0.0),
        types::AlgorithmStatus::Working,
    };
    ScriptedAlgorithm algorithm{missionConfig(), lidarConfig(), droneConfig(), *output, first_command};
    DroneControlImpl drone_control{
        droneConfig(),
        missionConfig(),
        lidar,
        gps,
        movement,
        *output,
        algorithm,
    };

    const types::DroneStepResult result = drone_control.step();

    EXPECT_EQ(result.status, types::DroneStepStatus::Continue);
    EXPECT_TRUE(algorithm.saw_null_latest_scan);
    EXPECT_EQ(lidar.scan_calls, 1);
    EXPECT_EQ(output->atVoxel(pos(20.0, 10.0, 10.0)), types::VoxelOccupancy::Empty);
}

TEST(MappingAlgorithm, HandlesNullScanAndEventuallyFinishes) {
    const types::MapConfig config = mapConfig(bounds(0.0, 20.0, 0.0, 20.0, 0.0, 20.0), 10.0);
    auto output = Map3DImpl::createEmpty(config);
    MappingAlgorithmImpl algorithm{missionConfig(), lidarConfig(), droneConfig(), *output};

    types::MappingStepCommand command =
        algorithm.nextStep(types::DroneState{pos(10.0, 10.0, 10.0), makeHeading(0.0), 0}, nullptr);
    EXPECT_EQ(command.status, types::AlgorithmStatus::Working);
    EXPECT_TRUE(command.scan_orientation.has_value());
    for (int i = 0; i < 10; ++i) {
        command = algorithm.nextStep(types::DroneState{pos(10.0, 10.0, 10.0), makeHeading(0.0), 0}, nullptr);
    }
    EXPECT_NE(command.status, types::AlgorithmStatus::Working);
}

TEST(MockLidar, CenterBeamDetectsObstacleNearMaximumRange) {
    const types::MapConfig config = mapConfig(bounds(0.0, 80.0, -20.0, 20.0, -20.0, 20.0), 10.0);
    auto hidden = Map3DImpl::createEmpty(config, types::VoxelOccupancy::Empty);
    hidden->set(pos(50.0, 0.0, 0.0), types::VoxelOccupancy::Occupied);
    MockGPS gps{pos(0.0, 0.0, 0.0), makeHeading(0.0), 10.0 * cm};
    MockLidar lidar{
        types::LidarConfigData{5.0 * cm, 60.0 * cm, 2.0 * cm, 1},
        *hidden,
        gps,
    };

    const types::LidarScanResult scan = lidar.scan(makeHeading(0.0));

    ASSERT_FALSE(scan.empty());
    EXPECT_NEAR(scan.front().distance.force_numerical_value_in(cm), 50.0, 1.1);
}

TEST(MapsComparison, IdenticalMapsReturnPerfectScoreAndDifferentMapsDoNot) {
    const types::MapConfig config = mapConfig(bounds(0.0, 20.0, 0.0, 20.0, 0.0, 10.0), 10.0);
    auto origin = Map3DImpl::createEmpty(config, types::VoxelOccupancy::Empty);
    auto same = Map3DImpl::createEmpty(config, types::VoxelOccupancy::Empty);
    auto different = Map3DImpl::createEmpty(config, types::VoxelOccupancy::Empty);
    origin->set(pos(0.0, 0.0, 0.0), types::VoxelOccupancy::Occupied);
    same->set(pos(0.0, 0.0, 0.0), types::VoxelOccupancy::Occupied);

    EXPECT_DOUBLE_EQ(MapsComparison::compareSingle(*origin, *same), 100.0);
    EXPECT_LT(MapsComparison::compareSingle(*origin, *different), 100.0);
}

TEST(MapsComparison, FileComparisonUsesConfiguredBoundaries) {
    const types::MapConfig config = mapConfig(bounds(0.0, 20.0, 0.0, 10.0, 0.0, 10.0), 10.0);
    auto origin = Map3DImpl::createEmpty(config, types::VoxelOccupancy::Empty);
    auto target = Map3DImpl::createEmpty(config, types::VoxelOccupancy::Empty);
    target->set(pos(10.0, 0.0, 0.0), types::VoxelOccupancy::Occupied);

    const std::filesystem::path output_dir =
        std::filesystem::temp_directory_path() / "drone_mapper_comparison_config_test";
    const std::filesystem::path origin_path = output_dir / "origin.npy";
    const std::filesystem::path target_path = output_dir / "target.npy";
    const std::filesystem::path config_path = output_dir / "comparison.yaml";
    origin->save(origin_path);
    target->save(target_path);

    std::ofstream config_file{config_path};
    config_file << "comparison_config:\n"
                << "  map_resolution_cm: 10\n"
                << "  map_axes_offset:\n"
                << "    x_offset: 0\n"
                << "    y_offset: 0\n"
                << "    height_offset: 0\n"
                << "  boundaries:\n"
                << "    x_boundary:\n"
                << "      min_cm: 0\n"
                << "      max_cm: 10\n"
                << "    y_boundary:\n"
                << "      min_cm: 0\n"
                << "      max_cm: 10\n"
                << "    height_boundary:\n"
                << "      min_cm: 0\n"
                << "      max_cm: 10\n";
    config_file.close();

    EXPECT_LT(MapsComparison::compareFiles(origin_path, target_path), 100.0);
    EXPECT_DOUBLE_EQ(MapsComparison::compareFiles(origin_path, target_path, config_path), 100.0);
}

TEST(MapsComparison, FileComparisonUsesRichOriginalTargetConfig) {
    const types::MapConfig config = mapConfig(bounds(0.0, 20.0, 0.0, 10.0, 0.0, 10.0), 10.0);
    auto origin = Map3DImpl::createEmpty(config, types::VoxelOccupancy::Empty);
    auto target = Map3DImpl::createEmpty(config, types::VoxelOccupancy::Empty);
    target->set(pos(10.0, 0.0, 0.0), types::VoxelOccupancy::Occupied);

    const std::filesystem::path output_dir =
        std::filesystem::temp_directory_path() / "drone_mapper_rich_comparison_config_test";
    const std::filesystem::path origin_path = output_dir / "origin.npy";
    const std::filesystem::path target_path = output_dir / "target.npy";
    const std::filesystem::path config_path = output_dir / "comparison.yaml";
    origin->save(origin_path);
    target->save(target_path);

    std::ofstream config_file{config_path};
    config_file << "comparison_config:\n"
                << "  original:\n"
                << "    map_res_cm: 10\n"
                << "    map_offset:\n"
                << "      x_offset: 0\n"
                << "      y_offset: 0\n"
                << "      height_offset: 0\n"
                << "    map_boundaries:\n"
                << "      x_boundary:\n"
                << "        min_cm: 0\n"
                << "        max_cm: 10\n"
                << "      y_boundary:\n"
                << "        min_cm: 0\n"
                << "        max_cm: 10\n"
                << "      height_boundary:\n"
                << "        min_cm: 0\n"
                << "        max_cm: 10\n"
                << "  target:\n"
                << "    map_res_cm: 10\n"
                << "    map_offset:\n"
                << "      x_offset: 0\n"
                << "      y_offset: 0\n"
                << "      height_offset: 0\n"
                << "    map_boundaries:\n"
                << "      x_boundary:\n"
                << "        min_cm: 0\n"
                << "        max_cm: 10\n"
                << "      y_boundary:\n"
                << "        min_cm: 0\n"
                << "        max_cm: 10\n"
                << "      height_boundary:\n"
                << "        min_cm: 0\n"
                << "        max_cm: 10\n";
    config_file.close();

    EXPECT_LT(MapsComparison::compareFiles(origin_path, target_path), 100.0);
    EXPECT_DOUBLE_EQ(MapsComparison::compareFiles(origin_path, target_path, config_path), 100.0);
}

} // namespace drone_mapper

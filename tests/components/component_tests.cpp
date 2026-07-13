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
#include <drone_mapper/ScanResultToVoxels.h>
#include <drone_mapper/SimulationManager.h>
#include <drone_mapper/SimulationRunImpl.h>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
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

class CountingReadMap final : public IMap3D {
public:
    CountingReadMap(const IMap3D& map, double far_x_cm)
        : map_(map), far_x_cm_(far_x_cm) {}

    types::VoxelOccupancy atVoxel(const Position3D& position) const override {
        ++at_voxel_calls;
        if (position.x.force_numerical_value_in(cm) > far_x_cm_) {
            ++far_voxel_calls;
        }
        return map_.atVoxel(position);
    }

    types::MapConfig getMapConfig() const override {
        ++get_map_config_calls;
        return map_.getMapConfig();
    }

    bool isInBounds(const Position3D& position) const override {
        return map_.isInBounds(position);
    }

    mutable std::size_t at_voxel_calls = 0;
    mutable std::size_t far_voxel_calls = 0;
    mutable std::size_t get_map_config_calls = 0;

private:
    const IMap3D& map_;
    double far_x_cm_ = 0.0;
};

class ThrowingComparisonMap final : public IMap3D {
public:
    explicit ThrowingComparisonMap(types::MapConfig config)
        : config_(std::move(config)) {}

    types::VoxelOccupancy atVoxel(const Position3D&) const override {
        throw std::runtime_error("deterministic comparison failure");
    }

    types::MapConfig getMapConfig() const override {
        return config_;
    }

    bool isInBounds(const Position3D&) const override {
        return true;
    }

private:
    types::MapConfig config_{};
};

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

class AlwaysContinueDroneControl final : public IDroneControl {
public:
    types::DroneStepResult step() override {
        ++step_calls;
        return types::DroneStepResult{types::DroneStepStatus::Continue, {}};
    }

    types::DroneState state() const override {
        return types::DroneState{pos(10.0, 10.0, 10.0), makeHeading(0.0), step_calls};
    }

    std::size_t step_calls = 0;
};

class PositionedDroneControl final : public IDroneControl {
public:
    explicit PositionedDroneControl(Position3D position,
                                    types::DroneStepResult step_result = {
                                        types::DroneStepStatus::Completed, {}})
        : position_(position),
          step_result_(std::move(step_result)) {}

    types::DroneStepResult step() override {
        ++step_calls;
        return step_result_;
    }

    types::DroneState state() const override {
        return types::DroneState{position_, makeHeading(0.0), step_calls};
    }

    std::size_t step_calls = 0;

private:
    Position3D position_{};
    types::DroneStepResult step_result_{};
};

[[nodiscard]] std::filesystem::path freshMissionTestDirectory(const std::string& name) {
    const std::filesystem::path directory =
        std::filesystem::temp_directory_path() / name;
    std::filesystem::remove_all(directory);
    std::filesystem::create_directories(directory);
    return directory;
}

[[nodiscard]] std::string readTextFile(const std::filesystem::path& path) {
    std::ifstream input{path};
    std::string contents;
    std::getline(input, contents, '\0');
    return contents;
}

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

class ThrowingMissionControl final : public IMissionControl {
public:
    types::MissionRunResult runMission() override {
        throw std::runtime_error("deterministic mission failure");
    }
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

TEST(SimulationRun, LogsCaughtMissionExceptionImmediately) {
    const types::MapConfig config = mapConfig(bounds(0.0, 20.0, 0.0, 20.0, 0.0, 20.0), 10.0);
    auto hidden = Map3DImpl::createEmpty(config, types::VoxelOccupancy::Empty);
    auto output = Map3DImpl::createEmpty(config, types::VoxelOccupancy::Empty);
    auto output_for_algorithm = Map3DImpl::createEmpty(config, types::VoxelOccupancy::Empty);
    const std::filesystem::path temporary =
        std::filesystem::temp_directory_path() / "simulation_run_exception_logging_test";
    std::filesystem::remove_all(temporary);
    const std::filesystem::path output_map_file =
        temporary / "output_results" / "run_000" / "output_map.npy";
    const std::filesystem::path error_log = output_map_file.parent_path() / "errors.log";

    SimulationRunImpl run{
        std::move(hidden),
        std::move(output),
        std::make_unique<NullGPS>(),
        std::make_unique<NullMovement>(),
        std::make_unique<NullLidar>(),
        std::make_unique<NullAlgorithm>(*output_for_algorithm),
        std::make_unique<NullDroneControl>(),
        std::make_unique<ThrowingMissionControl>(),
        types::SimulationConfigData{},
        missionConfig(),
        types::ResolutionRequestStatus::Accepted,
        output_map_file,
    };

    const types::SimulationResult result = run.run();

    ASSERT_EQ(result.mission_results.size(), 1U);
    const types::MissionRunResult& mission_result = result.mission_results.front();
    EXPECT_EQ(mission_result.status, types::MissionRunStatus::Error);
    EXPECT_DOUBLE_EQ(result.mission_score, -1.0);
    ASSERT_EQ(mission_result.errors.size(), 1U);
    EXPECT_EQ(mission_result.errors.front().code, "SIMULATION_RUN_ERROR");
    EXPECT_EQ(mission_result.errors.front().message, "deterministic mission failure");
    ASSERT_TRUE(std::filesystem::exists(error_log));
    const std::string log_contents = readTextFile(error_log);
    EXPECT_THAT(
        log_contents,
        testing::HasSubstr(
            "step=0 code=SIMULATION_RUN_ERROR message=\"deterministic mission failure\""));
    std::filesystem::remove_all(temporary);
}

TEST(SimulationRun, ReplacesCompletedMissionResultWhenComparisonThrows) {
    const types::MapConfig config = mapConfig(bounds(0.0, 10.0, 0.0, 10.0, 0.0, 10.0), 10.0);
    auto output = Map3DImpl::createEmpty(config, types::VoxelOccupancy::Empty);
    auto output_for_algorithm = Map3DImpl::createEmpty(config, types::VoxelOccupancy::Empty);
    auto mission = std::make_unique<CompletedMission>();
    CompletedMission* mission_ptr = mission.get();
    const std::filesystem::path temporary =
        std::filesystem::temp_directory_path() / "simulation_run_comparison_exception_test";
    std::filesystem::remove_all(temporary);
    const std::filesystem::path output_map_file =
        temporary / "output_results" / "run_000" / "output_map.npy";
    const std::filesystem::path error_log = output_map_file.parent_path() / "errors.log";

    SimulationRunImpl run{
        std::make_unique<ThrowingComparisonMap>(config),
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
        output_map_file,
    };

    const types::SimulationResult result = run.run();
    const bool log_exists = std::filesystem::exists(error_log);
    const std::string log_contents = log_exists ? readTextFile(error_log) : std::string{};
    std::filesystem::remove_all(temporary);

    EXPECT_TRUE(mission_ptr->called);
    EXPECT_DOUBLE_EQ(result.mission_score, -1.0);
    EXPECT_EQ(result.mission_results.size(), 1U);
    ASSERT_FALSE(result.mission_results.empty());
    EXPECT_EQ(result.mission_results.front().status, types::MissionRunStatus::Error);
    EXPECT_EQ(result.mission_results.front().steps, 1U);
    const types::MissionRunResult& error_result = result.mission_results.back();
    EXPECT_EQ(error_result.status, types::MissionRunStatus::Error);
    EXPECT_EQ(error_result.steps, 1U);
    ASSERT_EQ(error_result.errors.size(), 1U);
    EXPECT_EQ(error_result.errors.front().code, "SIMULATION_RUN_ERROR");
    EXPECT_EQ(error_result.errors.front().message, "deterministic comparison failure");
    EXPECT_TRUE(log_exists);
    EXPECT_THAT(
        log_contents,
        testing::HasSubstr(
            "step=0 code=SIMULATION_RUN_ERROR message=\"deterministic comparison failure\""));
}

TEST(SimulationRun, ScoresOnlyTheSharedOutputMissionDomain) {
    const types::MapConfig hidden_config =
        mapConfig(bounds(0.0, 30.0, 0.0, 10.0, 0.0, 10.0), 10.0);
    const types::MapConfig output_config =
        mapConfig(bounds(10.0, 30.0, 0.0, 10.0, 0.0, 10.0), 10.0);
    auto hidden = Map3DImpl::createEmpty(hidden_config, types::VoxelOccupancy::Empty);
    hidden->set(pos(0.0, 0.0, 0.0), types::VoxelOccupancy::Occupied);
    auto output = Map3DImpl::createEmpty(output_config, types::VoxelOccupancy::Empty);
    auto output_for_algorithm =
        Map3DImpl::createEmpty(output_config, types::VoxelOccupancy::Empty);
    auto mission = std::make_unique<CompletedMission>();
    types::MissionConfigData mission_config = missionConfig();
    mission_config.mission_bounds = output_config.boundaries;

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
        mission_config,
        types::ResolutionRequestStatus::Accepted,
        std::filesystem::temp_directory_path() / "simulation_run_shared_domain_output.npy",
    };

    const types::SimulationResult result = run.run();

    ASSERT_EQ(result.mission_results.size(), 1U);
    EXPECT_EQ(result.mission_results.front().status, types::MissionRunStatus::Completed);
    EXPECT_EQ(result.mission_results.front().steps, 1U);
    EXPECT_TRUE(result.mission_results.front().errors.empty());
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

TEST(MissionControl, ReportsMaxStepsAfterExactConfiguredNumberOfCalls) {
    const types::MapConfig config = mapConfig(bounds(0.0, 50.0, 0.0, 50.0, 0.0, 50.0), 10.0);
    auto hidden = Map3DImpl::createEmpty(config, types::VoxelOccupancy::Empty);
    auto output = Map3DImpl::createEmpty(config, types::VoxelOccupancy::Unmapped);
    types::MissionConfigData mission_config = missionConfig();
    mission_config.max_steps = 3;
    AlwaysContinueDroneControl drone_control;
    MissionControlImpl mission{
        mission_config,
        droneConfig(),
        *hidden,
        *output,
        drone_control,
        std::filesystem::temp_directory_path() / "mission_control_max_steps_output.npy",
    };

    const types::MissionRunResult result = mission.runMission();

    EXPECT_EQ(result.status, types::MissionRunStatus::MaxSteps);
    EXPECT_EQ(result.steps, mission_config.max_steps);
    EXPECT_EQ(drone_control.step_calls, mission_config.max_steps);
}

TEST(MissionControl, RejectsInitialSphereOverlappingAdjacentOccupiedVoxel) {
    const types::MappingBounds test_bounds = bounds(0.0, 40.0, 0.0, 40.0, 0.0, 40.0);
    const types::MapConfig config = mapConfig(test_bounds, 10.0);
    auto hidden = Map3DImpl::createEmpty(config, types::VoxelOccupancy::Empty);
    hidden->set(pos(15.0, 15.0, 15.0), types::VoxelOccupancy::Occupied);
    auto output = Map3DImpl::createEmpty(config, types::VoxelOccupancy::Unmapped);
    types::MissionConfigData mission_config = missionConfig();
    mission_config.mission_bounds = test_bounds;
    types::DroneConfigData drone_config = droneConfig();
    drone_config.radius = 2.0 * cm;
    PositionedDroneControl drone_control{pos(8.5, 15.0, 15.0)};
    const std::filesystem::path directory =
        freshMissionTestDirectory("mission_control_initial_sphere_overlap");
    const std::filesystem::path output_file = directory / "output.npy";
    MissionControlImpl mission{
        mission_config, drone_config, *hidden, *output, drone_control, output_file};

    const types::MissionRunResult result = mission.runMission();

    ASSERT_EQ(result.status, types::MissionRunStatus::Error);
    ASSERT_EQ(result.errors.size(), 1U);
    EXPECT_EQ(result.errors.front().code, "INVALID_INITIAL_POSITION");
    EXPECT_EQ(result.steps, 0U);
    EXPECT_EQ(drone_control.step_calls, 0U);
    EXPECT_TRUE(std::filesystem::exists(output_file));
    EXPECT_THAT(readTextFile(directory / "errors.log"),
                testing::HasSubstr("code=INVALID_INITIAL_POSITION"));
}

TEST(MissionControl, RejectsInitialSphereOutsideMissionBounds) {
    const types::MappingBounds hidden_bounds = bounds(0.0, 40.0, 0.0, 40.0, 0.0, 40.0);
    const types::MappingBounds mission_bounds = bounds(5.0, 35.0, 5.0, 35.0, 5.0, 35.0);
    auto hidden = Map3DImpl::createEmpty(
        mapConfig(hidden_bounds, 10.0), types::VoxelOccupancy::Empty);
    auto output = Map3DImpl::createEmpty(
        mapConfig(mission_bounds, 10.0), types::VoxelOccupancy::Unmapped);
    types::MissionConfigData mission_config = missionConfig();
    mission_config.mission_bounds = mission_bounds;
    types::DroneConfigData drone_config = droneConfig();
    drone_config.radius = 2.0 * cm;
    PositionedDroneControl drone_control{pos(6.0, 20.0, 20.0)};
    const std::filesystem::path directory =
        freshMissionTestDirectory("mission_control_initial_sphere_mission_bounds");
    const std::filesystem::path output_file = directory / "output.npy";
    MissionControlImpl mission{
        mission_config, drone_config, *hidden, *output, drone_control, output_file};

    const types::MissionRunResult result = mission.runMission();

    ASSERT_EQ(result.status, types::MissionRunStatus::Error);
    ASSERT_EQ(result.errors.size(), 1U);
    EXPECT_EQ(result.errors.front().code, "INVALID_INITIAL_POSITION");
    EXPECT_EQ(result.steps, 0U);
    EXPECT_EQ(drone_control.step_calls, 0U);
    EXPECT_TRUE(std::filesystem::exists(output_file));
    EXPECT_THAT(readTextFile(directory / "errors.log"),
                testing::HasSubstr("code=INVALID_INITIAL_POSITION"));
}

TEST(MissionControl, RejectsInitialSphereOutsideHiddenMapBounds) {
    const types::MappingBounds hidden_bounds = bounds(0.0, 40.0, 0.0, 40.0, 0.0, 40.0);
    const types::MappingBounds mission_bounds = bounds(-10.0, 50.0, -10.0, 50.0, -10.0, 50.0);
    auto hidden = Map3DImpl::createEmpty(
        mapConfig(hidden_bounds, 10.0), types::VoxelOccupancy::Empty);
    auto output = Map3DImpl::createEmpty(
        mapConfig(mission_bounds, 10.0), types::VoxelOccupancy::Unmapped);
    types::MissionConfigData mission_config = missionConfig();
    mission_config.mission_bounds = mission_bounds;
    types::DroneConfigData drone_config = droneConfig();
    drone_config.radius = 2.0 * cm;
    PositionedDroneControl drone_control{pos(1.0, 20.0, 20.0)};
    const std::filesystem::path directory =
        freshMissionTestDirectory("mission_control_initial_sphere_hidden_bounds");
    const std::filesystem::path output_file = directory / "output.npy";
    MissionControlImpl mission{
        mission_config, drone_config, *hidden, *output, drone_control, output_file};

    const types::MissionRunResult result = mission.runMission();

    ASSERT_EQ(result.status, types::MissionRunStatus::Error);
    ASSERT_EQ(result.errors.size(), 1U);
    EXPECT_EQ(result.errors.front().code, "INVALID_INITIAL_POSITION");
    EXPECT_EQ(result.steps, 0U);
    EXPECT_EQ(drone_control.step_calls, 0U);
    EXPECT_TRUE(std::filesystem::exists(output_file));
    EXPECT_THAT(readTextFile(directory / "errors.log"),
                testing::HasSubstr("code=INVALID_INITIAL_POSITION"));
}

TEST(MissionControl, AcceptsFullyClearInitialSphereInsideAllBounds) {
    const types::MappingBounds test_bounds = bounds(0.0, 40.0, 0.0, 40.0, 0.0, 40.0);
    const types::MapConfig config = mapConfig(test_bounds, 10.0);
    auto hidden = Map3DImpl::createEmpty(config, types::VoxelOccupancy::Empty);
    auto output = Map3DImpl::createEmpty(config, types::VoxelOccupancy::Unmapped);
    types::MissionConfigData mission_config = missionConfig();
    mission_config.mission_bounds = test_bounds;
    types::DroneConfigData drone_config = droneConfig();
    drone_config.radius = 2.0 * cm;
    PositionedDroneControl drone_control{pos(5.0, 15.0, 15.0)};
    const std::filesystem::path directory =
        freshMissionTestDirectory("mission_control_initial_sphere_clear");
    const std::filesystem::path output_file = directory / "output.npy";
    MissionControlImpl mission{
        mission_config, drone_config, *hidden, *output, drone_control, output_file};

    const types::MissionRunResult result = mission.runMission();

    EXPECT_EQ(result.status, types::MissionRunStatus::Completed);
    EXPECT_EQ(result.steps, 1U);
    EXPECT_EQ(drone_control.step_calls, 1U);
    EXPECT_TRUE(std::filesystem::exists(output_file));
    EXPECT_FALSE(std::filesystem::exists(directory / "errors.log"));
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

TEST(DroneControl, NaturallyFinishedAlgorithmProducesCompleted) {
    const types::MapConfig config = mapConfig(bounds(0.0, 80.0, 0.0, 80.0, 0.0, 80.0), 10.0);
    auto output = Map3DImpl::createEmpty(config, types::VoxelOccupancy::Unmapped);
    MockGPS gps{pos(10.0, 10.0, 10.0), makeHeading(0.0), 10.0 * cm};
    MockMovement movement{gps};
    FixedLidar lidar{lidarConfig()};
    const types::MappingStepCommand finished_command{
        std::nullopt,
        std::nullopt,
        types::AlgorithmStatus::Finished,
    };
    ScriptedAlgorithm algorithm{
        missionConfig(), lidarConfig(), droneConfig(), *output, finished_command};
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

    EXPECT_EQ(result.status, types::DroneStepStatus::Completed);
    EXPECT_EQ(algorithm.calls, 1);
    EXPECT_EQ(lidar.scan_calls, 0);
}

TEST(MockMovement, RejectsSmallSphereOverlappingAdjacentOccupiedVoxel) {
    const types::MapConfig config = mapConfig(bounds(0.0, 40.0, 0.0, 40.0, 0.0, 40.0), 10.0);
    auto hidden = Map3DImpl::createEmpty(config, types::VoxelOccupancy::Empty);
    hidden->set(pos(15.0, 15.0, 15.0), types::VoxelOccupancy::Occupied);
    MockGPS gps{pos(5.0, 15.0, 15.0), makeHeading(0.0), 10.0 * cm};
    MockMovement movement{gps, *hidden, 2.0 * cm};

    const types::MovementResult result = movement.advance(3.5 * cm);

    EXPECT_FALSE(result.success);
    EXPECT_DOUBLE_EQ(gps.position().x.force_numerical_value_in(cm), 5.0);
}

TEST(MockMovement, RejectsSmallSphereTangentToAdjacentOccupiedVoxel) {
    const types::MapConfig config = mapConfig(bounds(0.0, 40.0, 0.0, 40.0, 0.0, 40.0), 10.0);
    auto hidden = Map3DImpl::createEmpty(config, types::VoxelOccupancy::Empty);
    hidden->set(pos(15.0, 15.0, 15.0), types::VoxelOccupancy::Occupied);
    MockGPS gps{pos(5.0, 15.0, 15.0), makeHeading(0.0), 10.0 * cm};
    MockMovement movement{gps, *hidden, 2.0 * cm};

    const types::MovementResult result = movement.advance(3.0 * cm);

    EXPECT_FALSE(result.success);
    EXPECT_DOUBLE_EQ(gps.position().x.force_numerical_value_in(cm), 5.0);
}

TEST(MockMovement, AcceptsSmallSphereWhenNeighboringVoxelsAreClear) {
    const types::MapConfig config = mapConfig(bounds(0.0, 40.0, 0.0, 40.0, 0.0, 40.0), 10.0);
    auto hidden = Map3DImpl::createEmpty(config, types::VoxelOccupancy::Empty);
    MockGPS gps{pos(5.0, 15.0, 15.0), makeHeading(0.0), 10.0 * cm};
    MockMovement movement{gps, *hidden, 2.0 * cm};

    const types::MovementResult result = movement.advance(3.5 * cm);

    EXPECT_TRUE(result.success);
    EXPECT_DOUBLE_EQ(gps.position().x.force_numerical_value_in(cm), 8.5);
}

TEST(MockMovement, RejectsSmallSphereSweptThroughOccupiedVoxel) {
    const types::MapConfig config = mapConfig(bounds(0.0, 40.0, 0.0, 40.0, 0.0, 40.0), 10.0);
    auto hidden = Map3DImpl::createEmpty(config, types::VoxelOccupancy::Empty);
    hidden->set(pos(15.0, 15.0, 15.0), types::VoxelOccupancy::Occupied);
    MockGPS gps{pos(7.9, 11.0, 15.0), makeHeading(-45.0), 10.0 * cm};
    MockMovement movement{gps, *hidden, 2.0 * cm};

    const types::MovementResult result = movement.advance(3.1 * std::sqrt(2.0) * cm);

    EXPECT_FALSE(result.success);
    EXPECT_DOUBLE_EQ(gps.position().x.force_numerical_value_in(cm), 7.9);
    EXPECT_DOUBLE_EQ(gps.position().y.force_numerical_value_in(cm), 11.0);
}

TEST(MappingAlgorithm, DoesNotClaimCompletionAtMissionStepLimit) {
    const types::MapConfig config = mapConfig(bounds(0.0, 20.0, 0.0, 20.0, 0.0, 20.0), 10.0);
    auto output = Map3DImpl::createEmpty(config);
    MappingAlgorithmImpl algorithm{missionConfig(), lidarConfig(), droneConfig(), *output};

    const types::MappingStepCommand command = algorithm.nextStep(
        types::DroneState{pos(10.0, 10.0, 10.0), makeHeading(0.0), missionConfig().max_steps - 1},
        nullptr);

    EXPECT_EQ(command.status, types::AlgorithmStatus::Working);
}

TEST(MappingAlgorithm, FullyMappedMapReturnsFinished) {
    const types::MappingBounds mission_bounds = bounds(0.0, 10.0, 0.0, 10.0, 0.0, 10.0);
    const types::MapConfig config = mapConfig(mission_bounds, 10.0);
    auto output = Map3DImpl::createEmpty(config, types::VoxelOccupancy::Empty);
    const types::MissionConfigData mission{10, 10.0 * cm, 1.0, mission_bounds};
    MappingAlgorithmImpl algorithm{mission, lidarConfig(), droneConfig(), *output};

    const types::MappingStepCommand command = algorithm.nextStep(
        types::DroneState{pos(5.0, 5.0, 5.0), makeHeading(0.0), 0}, nullptr);

    EXPECT_EQ(command.status, types::AlgorithmStatus::Finished);
}

TEST(MappingAlgorithm, UnreachableUnknownVoxelReturnsFinishedWithUnmappableVoxels) {
    const types::MappingBounds mission_bounds = bounds(0.0, 20.0, 0.0, 10.0, 0.0, 10.0);
    const types::MapConfig config = mapConfig(mission_bounds, 10.0);
    auto output = Map3DImpl::createEmpty(config, types::VoxelOccupancy::Occupied);
    output->set(pos(5.0, 5.0, 5.0), types::VoxelOccupancy::Empty);
    output->set(pos(15.0, 5.0, 5.0), types::VoxelOccupancy::Unmapped);
    const types::MissionConfigData mission{10, 10.0 * cm, 1.0, mission_bounds};
    types::LidarConfigData lidar = lidarConfig();
    lidar.fov_circles = 0;
    MappingAlgorithmImpl algorithm{mission, lidar, droneConfig(), *output};

    const types::MappingStepCommand command = algorithm.nextStep(
        types::DroneState{pos(5.0, 5.0, 5.0), makeHeading(0.0), 0}, nullptr);

    EXPECT_EQ(command.status, types::AlgorithmStatus::FinishedWithUnmappableVoxels);
}

TEST(MappingAlgorithm, FineResolutionScansVerticalDiagonalsAndFindsMove) {
    const types::MappingBounds mission_bounds = bounds(0.0, 55.0, 0.0, 55.0, 0.0, 55.0);
    const types::MapConfig config = mapConfig(mission_bounds, 5.0);
    auto output = Map3DImpl::createEmpty(config);
    const types::MissionConfigData mission{200, 5.0 * cm, 1.0, mission_bounds};
    const types::DroneConfigData drone{
        3.0 * cm,
        90.0 * horizontal_angle[deg],
        20.0 * cm,
        20.0 * cm,
    };
    const types::LidarConfigData lidar{5.0 * cm, 25.0 * cm, 2.0 * cm, 1};
    MappingAlgorithmImpl algorithm{mission, lidar, drone, *output};
    types::DroneState state{pos(27.5, 27.5, 27.5), makeHeading(90.0), 0};
    std::optional<types::LidarScanResult> latest_scan;
    bool saw_vertical_diagonal_scan = false;
    bool saw_movement = false;
    bool finished_before_movement = false;

    for (std::size_t step = 0; step < 80; ++step) {
        state.step_index = step;
        const types::MappingStepCommand command =
            algorithm.nextStep(state, latest_scan ? &(*latest_scan) : nullptr);
        if (command.status != types::AlgorithmStatus::Working) {
            finished_before_movement = true;
            break;
        }
        if (command.movement.has_value()) {
            saw_movement = true;
            break;
        }
        ASSERT_TRUE(command.scan_orientation.has_value());

        const double altitude = std::abs(
            command.scan_orientation->altitude.force_numerical_value_in(deg));
        if (altitude > 1.0 && altitude < 89.0) {
            saw_vertical_diagonal_scan = true;
        }

        latest_scan = types::LidarScanResult{
            types::LidarHit{
                std::numeric_limits<double>::max() * cm,
                *command.scan_orientation,
            },
        };
        ScanResultToVoxels::applyToMap(
            *output, state.position, state.heading, *latest_scan, lidar);
    }

    EXPECT_TRUE(saw_vertical_diagonal_scan);
    EXPECT_TRUE(saw_movement);
    EXPECT_FALSE(finished_before_movement);
}

TEST(MappingAlgorithm, StopsBaseScanningAsSoonAsNeighborBecomesSafe) {
    const types::MappingBounds mission_bounds = bounds(0.0, 55.0, 0.0, 55.0, 0.0, 55.0);
    const types::MapConfig config = mapConfig(mission_bounds, 5.0);
    auto output = Map3DImpl::createEmpty(config, types::VoxelOccupancy::Occupied);
    const types::MissionConfigData mission{100, 5.0 * cm, 1.0, mission_bounds};
    const types::DroneConfigData drone{
        3.0 * cm,
        90.0 * horizontal_angle[deg],
        20.0 * cm,
        20.0 * cm,
    };
    const types::LidarConfigData lidar{5.0 * cm, 25.0 * cm, 2.0 * cm, 1};
    const Position3D current_position = pos(27.5, 27.5, 27.5);
    const Position3D forward_center = pos(32.5, 27.5, 27.5);

    output->set(current_position, types::VoxelOccupancy::Empty);
    output->set(forward_center, types::VoxelOccupancy::Unmapped);
    for (const Position3D& empty : std::vector<Position3D>{
             pos(37.5, 27.5, 27.5),
             pos(32.5, 22.5, 27.5),
             pos(32.5, 32.5, 27.5),
             pos(32.5, 27.5, 22.5),
             pos(32.5, 27.5, 32.5),
         }) {
        output->set(empty, types::VoxelOccupancy::Empty);
    }

    MappingAlgorithmImpl algorithm{mission, lidar, drone, *output};
    const types::DroneState state{current_position, makeHeading(0.0), 0};
    const types::MappingStepCommand first_command = algorithm.nextStep(state, nullptr);
    ASSERT_TRUE(first_command.scan_orientation.has_value());
    EXPECT_NEAR(first_command.scan_orientation->horizontal.force_numerical_value_in(deg), 0.0, 1.0e-6);
    EXPECT_NEAR(first_command.scan_orientation->altitude.force_numerical_value_in(deg), 0.0, 1.0e-6);

    output->set(forward_center, types::VoxelOccupancy::Empty);
    output->set(pos(42.5, 27.5, 27.5), types::VoxelOccupancy::Unmapped);
    const types::LidarScanResult latest_scan{
        types::LidarHit{std::numeric_limits<double>::max() * cm, *first_command.scan_orientation},
    };
    const types::MappingStepCommand next_command = algorithm.nextStep(state, &latest_scan);

    EXPECT_TRUE(next_command.movement.has_value());
    EXPECT_FALSE(next_command.scan_orientation.has_value());
}

TEST(MappingAlgorithm, ReprioritizesRemainingBaseDirectionsAfterScan) {
    const types::MappingBounds mission_bounds = bounds(0.0, 55.0, 0.0, 55.0, 0.0, 55.0);
    const types::MapConfig config = mapConfig(mission_bounds, 5.0);
    auto output = Map3DImpl::createEmpty(config, types::VoxelOccupancy::Occupied);
    const types::MissionConfigData mission{100, 5.0 * cm, 1.0, mission_bounds};
    const types::DroneConfigData drone{
        3.0 * cm,
        90.0 * horizontal_angle[deg],
        20.0 * cm,
        20.0 * cm,
    };
    const types::LidarConfigData lidar{5.0 * cm, 25.0 * cm, 2.0 * cm, 1};
    const Position3D current_position = pos(27.5, 27.5, 27.5);
    const std::vector<Position3D> forward_unknown{
        pos(32.5, 27.5, 27.5),
        pos(37.5, 27.5, 27.5),
        pos(42.5, 27.5, 27.5),
    };

    output->set(current_position, types::VoxelOccupancy::Empty);
    for (const Position3D& unknown : forward_unknown) {
        output->set(unknown, types::VoxelOccupancy::Unmapped);
    }
    output->set(pos(27.5, 32.5, 27.5), types::VoxelOccupancy::Unmapped);
    output->set(pos(27.5, 37.5, 27.5), types::VoxelOccupancy::Unmapped);

    MappingAlgorithmImpl algorithm{mission, lidar, drone, *output};
    const types::DroneState state{current_position, makeHeading(0.0), 0};
    const types::MappingStepCommand first_command = algorithm.nextStep(state, nullptr);
    ASSERT_TRUE(first_command.scan_orientation.has_value());
    EXPECT_NEAR(first_command.scan_orientation->horizontal.force_numerical_value_in(deg), 0.0, 1.0e-6);

    for (const Position3D& mapped : forward_unknown) {
        output->set(mapped, types::VoxelOccupancy::Occupied);
    }
    const types::LidarScanResult latest_scan{
        types::LidarHit{std::numeric_limits<double>::max() * cm, *first_command.scan_orientation},
    };
    const types::MappingStepCommand next_command = algorithm.nextStep(state, &latest_scan);

    ASSERT_TRUE(next_command.scan_orientation.has_value());
    EXPECT_NEAR(next_command.scan_orientation->horizontal.force_numerical_value_in(deg), 90.0, 1.0e-6);
}

TEST(MappingAlgorithm, FieldOfViewPotentialIncludesOffAxisUnknownVoxels) {
    const types::MappingBounds mission_bounds = bounds(0.0, 55.0, 0.0, 55.0, 0.0, 55.0);
    const types::MapConfig config = mapConfig(mission_bounds, 5.0);
    auto output = Map3DImpl::createEmpty(config, types::VoxelOccupancy::Occupied);
    const types::MissionConfigData mission{100, 5.0 * cm, 1.0, mission_bounds};
    const types::DroneConfigData drone{
        3.0 * cm,
        90.0 * horizontal_angle[deg],
        20.0 * cm,
        20.0 * cm,
    };
    const types::LidarConfigData lidar{5.0 * cm, 25.0 * cm, 5.0 * cm, 2};
    const Position3D current_position = pos(27.5, 27.5, 27.5);
    output->set(current_position, types::VoxelOccupancy::Empty);
    output->set(pos(32.5, 32.5, 27.5), types::VoxelOccupancy::Unmapped);

    MappingAlgorithmImpl algorithm{mission, lidar, drone, *output};
    const types::MappingStepCommand command =
        algorithm.nextStep(types::DroneState{current_position, makeHeading(0.0), 0}, nullptr);

    ASSERT_TRUE(command.scan_orientation.has_value());
    EXPECT_NEAR(command.scan_orientation->horizontal.force_numerical_value_in(deg), 0.0, 1.0e-6);
}

TEST(MappingAlgorithm, OccupiedVoxelBlocksScanPotentialBehindIt) {
    const types::MappingBounds mission_bounds = bounds(0.0, 40.0, 0.0, 10.0, 0.0, 10.0);
    const types::MapConfig config = mapConfig(mission_bounds, 10.0);
    auto output = Map3DImpl::createEmpty(config, types::VoxelOccupancy::Occupied);
    const Position3D current_position = pos(5.0, 5.0, 5.0);
    output->set(current_position, types::VoxelOccupancy::Empty);
    output->set(pos(25.0, 5.0, 5.0), types::VoxelOccupancy::Unmapped);
    const types::MissionConfigData mission{100, 10.0 * cm, 1.0, mission_bounds};
    const types::LidarConfigData lidar{5.0 * cm, 40.0 * cm, 2.0 * cm, 1};
    MappingAlgorithmImpl algorithm{mission, lidar, droneConfig(), *output};

    const types::MappingStepCommand command = algorithm.nextStep(
        types::DroneState{current_position, makeHeading(0.0), 0}, nullptr);

    EXPECT_FALSE(command.scan_orientation.has_value());
    EXPECT_FALSE(command.movement.has_value());
    EXPECT_EQ(command.status, types::AlgorithmStatus::FinishedWithUnmappableVoxels);
}

TEST(MappingAlgorithm, ScanCapPreventsMovementToNonActionableCoverageCell) {
    const types::MappingBounds mission_bounds = bounds(0.0, 20.0, 0.0, 5.0, 0.0, 5.0);
    const types::MapConfig config = mapConfig(mission_bounds, 5.0);
    auto output = Map3DImpl::createEmpty(config, types::VoxelOccupancy::Occupied);
    const Position3D first_position = pos(2.5, 2.5, 2.5);
    output->set(first_position, types::VoxelOccupancy::Empty);
    output->set(pos(7.5, 2.5, 2.5), types::VoxelOccupancy::Unmapped);
    const types::MissionConfigData mission{12, 5.0 * cm, 1.0, mission_bounds};
    const types::LidarConfigData lidar{5.0 * cm, 20.0 * cm, 2.0 * cm, 1};
    MappingAlgorithmImpl algorithm{mission, lidar, droneConfig(), *output};

    const types::MappingStepCommand first_command = algorithm.nextStep(
        types::DroneState{first_position, makeHeading(0.0), 0}, nullptr);
    ASSERT_TRUE(first_command.scan_orientation.has_value());

    output->set(pos(7.5, 2.5, 2.5), types::VoxelOccupancy::Empty);
    output->set(pos(12.5, 2.5, 2.5), types::VoxelOccupancy::Empty);
    output->set(pos(17.5, 2.5, 2.5), types::VoxelOccupancy::Unmapped);
    const types::DroneState next_cell{pos(7.5, 2.5, 2.5), makeHeading(0.0), 1};

    const types::MappingStepCommand command = algorithm.nextStep(next_cell, nullptr);

    EXPECT_FALSE(command.movement.has_value());
    EXPECT_FALSE(command.scan_orientation.has_value());
    EXPECT_EQ(command.status, types::AlgorithmStatus::FinishedWithUnmappableVoxels);
}

TEST(MappingAlgorithm, TraversesNonActionableCellTowardActionableCoverageTarget) {
    const types::MappingBounds mission_bounds = bounds(0.0, 20.0, 0.0, 5.0, 0.0, 5.0);
    const types::MapConfig config = mapConfig(mission_bounds, 5.0);
    auto output = Map3DImpl::createEmpty(config, types::VoxelOccupancy::Occupied);
    output->set(pos(2.5, 2.5, 2.5), types::VoxelOccupancy::Empty);
    output->set(pos(7.5, 2.5, 2.5), types::VoxelOccupancy::Empty);
    output->set(pos(12.5, 2.5, 2.5), types::VoxelOccupancy::Empty);
    output->set(pos(17.5, 2.5, 2.5), types::VoxelOccupancy::Unmapped);
    const types::MissionConfigData mission{100, 5.0 * cm, 1.0, mission_bounds};
    const types::LidarConfigData lidar{2.5 * cm, 5.0 * cm, 1.0 * cm, 1};
    MappingAlgorithmImpl algorithm{mission, lidar, droneConfig(), *output};

    const types::MappingStepCommand command = algorithm.nextStep(
        types::DroneState{pos(2.5, 2.5, 2.5), makeHeading(0.0), 0}, nullptr);

    ASSERT_TRUE(command.movement.has_value());
    EXPECT_EQ(command.movement->type, types::MovementCommandType::Advance);
    EXPECT_EQ(command.status, types::AlgorithmStatus::Working);

    const types::MappingStepCommand second_command = algorithm.nextStep(
        types::DroneState{pos(7.5, 2.5, 2.5), makeHeading(0.0), 1}, nullptr);
    ASSERT_TRUE(second_command.movement.has_value());
    EXPECT_EQ(second_command.movement->type, types::MovementCommandType::Advance);

    const types::MappingStepCommand destination_command = algorithm.nextStep(
        types::DroneState{pos(12.5, 2.5, 2.5), makeHeading(0.0), 2}, nullptr);
    EXPECT_TRUE(destination_command.scan_orientation.has_value());
    EXPECT_EQ(destination_command.status, types::AlgorithmStatus::Working);
}

TEST(MappingAlgorithm, ActionabilityExistenceQueryFindsUsefulBaseScanWithoutFullRanking) {
    const types::MappingBounds mission_bounds = bounds(0.0, 70.0, 0.0, 25.0, 0.0, 25.0);
    const types::MapConfig config = mapConfig(mission_bounds, 5.0);
    auto output = Map3DImpl::createEmpty(config, types::VoxelOccupancy::Occupied);
    const Position3D current_position = pos(12.5, 12.5, 12.5);
    const Position3D next_position = pos(17.5, 12.5, 12.5);
    output->set(current_position, types::VoxelOccupancy::Empty);
    output->set(next_position, types::VoxelOccupancy::Empty);
    for (double x = 22.5; x < 70.0; x += 5.0) {
        output->set(pos(x, 12.5, 12.5), types::VoxelOccupancy::Unmapped);
    }

    const types::MissionConfigData mission{100, 5.0 * cm, 1.0, mission_bounds};
    const types::LidarConfigData lidar{5.0 * cm, 50.0 * cm, 2.0 * cm, 1};
    CountingReadMap counting_map{*output, 30.0};
    MappingAlgorithmImpl algorithm{mission, lidar, droneConfig(), counting_map};

    const types::MappingStepCommand command = algorithm.nextStep(
        types::DroneState{current_position, makeHeading(0.0), 0}, nullptr);

    ASSERT_TRUE(command.movement.has_value());
    EXPECT_EQ(command.movement->type, types::MovementCommandType::Advance);
    EXPECT_EQ(command.status, types::AlgorithmStatus::Working);
    EXPECT_EQ(counting_map.far_voxel_calls, 0U);
}

TEST(MappingAlgorithm, ScanPotentialReusesInvariantMapGeometryAcrossOverlappingBeams) {
    const types::MappingBounds mission_bounds = bounds(0.0, 100.0, 0.0, 55.0, 0.0, 55.0);
    const types::MapConfig config = mapConfig(mission_bounds, 5.0);
    auto output = Map3DImpl::createEmpty(config, types::VoxelOccupancy::Occupied);
    const Position3D current_position = pos(32.5, 27.5, 27.5);
    output->set(current_position, types::VoxelOccupancy::Empty);
    for (double x = 37.5; x <= 72.5; x += 5.0) {
        output->set(pos(x, 27.5, 27.5), types::VoxelOccupancy::Unmapped);
    }

    const types::MissionConfigData mission{100, 5.0 * cm, 1.0, mission_bounds};
    const types::LidarConfigData lidar{5.0 * cm, 40.0 * cm, 2.0 * cm, 2};
    CountingReadMap counting_map{*output, 1000.0};
    MappingAlgorithmImpl algorithm{mission, lidar, droneConfig(), counting_map};

    const types::MappingStepCommand command = algorithm.nextStep(
        types::DroneState{current_position, makeHeading(0.0), 0}, nullptr);

    ASSERT_TRUE(command.scan_orientation.has_value());
    EXPECT_FALSE(command.movement.has_value());
    EXPECT_EQ(command.status, types::AlgorithmStatus::Working);
    EXPECT_NEAR(
        command.scan_orientation->horizontal.force_numerical_value_in(deg), 0.0, 1.0e-6);
    EXPECT_NEAR(
        command.scan_orientation->altitude.force_numerical_value_in(deg), 0.0, 1.0e-6);
    EXPECT_LE(counting_map.get_map_config_calls, 525U);
}

TEST(MappingAlgorithm, SkipsBaseDirectionsWithoutUsefulPotential) {
    const types::MappingBounds mission_bounds = bounds(0.0, 55.0, 0.0, 55.0, 0.0, 55.0);
    const types::MapConfig config = mapConfig(mission_bounds, 5.0);
    auto output = Map3DImpl::createEmpty(config, types::VoxelOccupancy::Occupied);
    const types::MissionConfigData mission{100, 5.0 * cm, 1.0, mission_bounds};
    const types::DroneConfigData drone{
        3.0 * cm,
        90.0 * horizontal_angle[deg],
        20.0 * cm,
        20.0 * cm,
    };
    const types::LidarConfigData lidar{5.0 * cm, 25.0 * cm, 2.0 * cm, 1};
    const Position3D current_position = pos(27.5, 27.5, 27.5);
    output->set(current_position, types::VoxelOccupancy::Empty);

    MappingAlgorithmImpl algorithm{mission, lidar, drone, *output};
    const types::MappingStepCommand command =
        algorithm.nextStep(types::DroneState{current_position, makeHeading(0.0), 0}, nullptr);

    EXPECT_FALSE(command.scan_orientation.has_value());
    EXPECT_FALSE(command.movement.has_value());
    EXPECT_NE(command.status, types::AlgorithmStatus::Working);
}

TEST(MappingAlgorithm, SkipsUselessBaseScansAndRoundRobinsAdaptiveScans) {
    const types::MappingBounds mission_bounds = bounds(0.0, 55.0, 0.0, 55.0, 0.0, 55.0);
    const types::MapConfig config = mapConfig(mission_bounds, 5.0);
    auto output = Map3DImpl::createEmpty(config, types::VoxelOccupancy::Occupied);
    const types::MissionConfigData mission{100, 5.0 * cm, 1.0, mission_bounds};
    const types::DroneConfigData drone{
        3.0 * cm,
        90.0 * horizontal_angle[deg],
        20.0 * cm,
        20.0 * cm,
    };
    const types::LidarConfigData lidar{5.0 * cm, 25.0 * cm, 2.0 * cm, 1};
    const Position3D current_position = pos(27.5, 27.5, 27.5);

    output->set(current_position, types::VoxelOccupancy::Empty);
    for (const Position3D& empty : std::vector<Position3D>{
             pos(32.5, 27.5, 27.5),
             pos(37.5, 27.5, 27.5),
             pos(32.5, 22.5, 27.5),
             pos(32.5, 32.5, 27.5),
             pos(32.5, 27.5, 22.5),
             pos(22.5, 27.5, 27.5),
             pos(17.5, 27.5, 27.5),
             pos(22.5, 22.5, 27.5),
             pos(22.5, 32.5, 27.5),
             pos(22.5, 27.5, 22.5),
             pos(27.5, 22.5, 27.5),
             pos(27.5, 32.5, 27.5),
             pos(27.5, 27.5, 22.5),
             pos(27.5, 27.5, 32.5),
         }) {
        output->set(empty, types::VoxelOccupancy::Empty);
    }
    const Position3D forward_blocker = pos(32.5, 27.5, 32.5);
    const Position3D backward_blocker = pos(22.5, 27.5, 32.5);
    output->set(forward_blocker, types::VoxelOccupancy::Unmapped);
    output->set(backward_blocker, types::VoxelOccupancy::Unmapped);

    MappingAlgorithmImpl algorithm{mission, lidar, drone, *output};
    const types::DroneState state{current_position, makeHeading(0.0), 0};
    const types::MappingStepCommand first_adaptive_scan = algorithm.nextStep(state, nullptr);
    ASSERT_TRUE(first_adaptive_scan.scan_orientation.has_value());
    EXPECT_FALSE(first_adaptive_scan.movement.has_value());
    EXPECT_NEAR(
        first_adaptive_scan.scan_orientation->horizontal.force_numerical_value_in(deg), 0.0, 1.0e-6);
    EXPECT_NEAR(
        first_adaptive_scan.scan_orientation->altitude.force_numerical_value_in(deg), 45.0, 1.0e-6);

    const types::LidarScanResult latest_scan{
        types::LidarHit{
            std::numeric_limits<double>::max() * cm,
            *first_adaptive_scan.scan_orientation,
        },
    };
    const types::MappingStepCommand next_adaptive_scan = algorithm.nextStep(state, &latest_scan);
    ASSERT_TRUE(next_adaptive_scan.scan_orientation.has_value());
    EXPECT_NEAR(
        next_adaptive_scan.scan_orientation->horizontal.force_numerical_value_in(deg), 180.0, 1.0e-6);
    EXPECT_NEAR(
        next_adaptive_scan.scan_orientation->altitude.force_numerical_value_in(deg), 45.0, 1.0e-6);

    output->set(backward_blocker, types::VoxelOccupancy::Empty);
    output->set(pos(12.5, 27.5, 27.5), types::VoxelOccupancy::Unmapped);
    const types::MappingStepCommand next_command = algorithm.nextStep(state, nullptr);

    ASSERT_TRUE(next_command.movement.has_value());
    EXPECT_EQ(output->atVoxel(forward_blocker), types::VoxelOccupancy::Unmapped);
}

TEST(MappingAlgorithm, TargetsVisibleUnresolvedSurfaceThatDoesNotBlockMovement) {
    const types::MappingBounds mission_bounds = bounds(0.0, 55.0, 0.0, 55.0, 0.0, 55.0);
    const types::MapConfig config = mapConfig(mission_bounds, 5.0);
    auto output = Map3DImpl::createEmpty(config, types::VoxelOccupancy::Occupied);
    const types::MissionConfigData mission{100, 5.0 * cm, 1.0, mission_bounds};
    const types::DroneConfigData drone{
        1.0 * cm,
        90.0 * horizontal_angle[deg],
        20.0 * cm,
        20.0 * cm,
    };
    const types::LidarConfigData lidar{5.0 * cm, 25.0 * cm, 2.0 * cm, 1};
    const Position3D current_position = pos(27.5, 27.5, 17.5);
    const std::vector<Position3D> safe_neighbors{
        pos(32.5, 27.5, 17.5),
        pos(22.5, 27.5, 17.5),
        pos(27.5, 32.5, 17.5),
        pos(27.5, 22.5, 17.5),
        pos(27.5, 27.5, 22.5),
        pos(27.5, 27.5, 12.5),
    };
    const Position3D unresolved_floor = pos(37.5, 32.5, 2.5);

    output->set(current_position, types::VoxelOccupancy::Empty);
    for (const Position3D& neighbor : safe_neighbors) {
        output->set(neighbor, types::VoxelOccupancy::Empty);
    }
    for (const Position3D& visible_path_cell : std::vector<Position3D>{
             pos(32.5, 27.5, 12.5),
             pos(32.5, 32.5, 7.5),
             pos(37.5, 32.5, 7.5),
         }) {
        output->set(visible_path_cell, types::VoxelOccupancy::Empty);
    }

    MappingAlgorithmImpl algorithm{mission, lidar, drone, *output};
    for (const Position3D& neighbor : safe_neighbors) {
        const types::MappingStepCommand visit_command = algorithm.nextStep(
            types::DroneState{neighbor, makeHeading(0.0), 0}, nullptr);
        EXPECT_NE(visit_command.status, types::AlgorithmStatus::Working);
    }
    output->set(unresolved_floor, types::VoxelOccupancy::Unmapped);

    const types::MappingStepCommand command = algorithm.nextStep(
        types::DroneState{current_position, makeHeading(0.0), 0}, nullptr);

    ASSERT_TRUE(command.scan_orientation.has_value());
    EXPECT_FALSE(command.movement.has_value());
    EXPECT_EQ(command.status, types::AlgorithmStatus::Working);
    EXPECT_NEAR(
        command.scan_orientation->horizontal.force_numerical_value_in(deg), 26.565051, 1.0e-5);
    EXPECT_NEAR(
        command.scan_orientation->altitude.force_numerical_value_in(deg), -53.300775, 1.0e-5);
}

TEST(MappingAlgorithm, ScanDirectionsAreRelativeToDroneHeading) {
    const types::MapConfig config = mapConfig(bounds(0.0, 50.0, 0.0, 50.0, 0.0, 50.0), 5.0);
    auto output = Map3DImpl::createEmpty(config, types::VoxelOccupancy::Occupied);
    const types::MissionConfigData mission{
        20,
        5.0 * cm,
        1.0,
        config.boundaries,
    };
    output->set(pos(22.5, 22.5, 22.5), types::VoxelOccupancy::Empty);
    for (double y = 27.5; y < 50.0; y += 5.0) {
        output->set(pos(22.5, y, 22.5), types::VoxelOccupancy::Unmapped);
    }
    MappingAlgorithmImpl algorithm{mission, lidarConfig(), droneConfig(), *output};

    const types::MappingStepCommand command = algorithm.nextStep(
        types::DroneState{pos(22.5, 22.5, 22.5), makeHeading(90.0), 0}, nullptr);

    ASSERT_TRUE(command.scan_orientation.has_value());
    EXPECT_NEAR(command.scan_orientation->horizontal.force_numerical_value_in(deg), 0.0, 1.0e-6);
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

TEST(MapsComparison, DifferentBoundariesWithIdenticalSharedVoxelsScorePerfectly) {
    const types::MapConfig origin_config =
        mapConfig(bounds(0.0, 30.0, 0.0, 10.0, 0.0, 10.0), 10.0);
    const types::MapConfig target_config =
        mapConfig(bounds(10.0, 30.0, 0.0, 10.0, 0.0, 10.0), 10.0);
    auto origin = Map3DImpl::createEmpty(origin_config, types::VoxelOccupancy::Empty);
    auto target = Map3DImpl::createEmpty(target_config, types::VoxelOccupancy::Empty);

    EXPECT_DOUBLE_EQ(MapsComparison::compareSingle(*origin, *target), 100.0);
}

TEST(MapsComparison, DifferenceInsideSharedDomainReducesScore) {
    const types::MapConfig origin_config =
        mapConfig(bounds(0.0, 30.0, 0.0, 10.0, 0.0, 10.0), 10.0);
    const types::MapConfig target_config =
        mapConfig(bounds(10.0, 30.0, 0.0, 10.0, 0.0, 10.0), 10.0);
    auto origin = Map3DImpl::createEmpty(origin_config, types::VoxelOccupancy::Empty);
    auto target = Map3DImpl::createEmpty(target_config, types::VoxelOccupancy::Empty);
    origin->set(pos(20.0, 0.0, 0.0), types::VoxelOccupancy::Occupied);

    const double score = MapsComparison::compareSingle(*origin, *target);

    EXPECT_DOUBLE_EQ(score, 50.0);
    EXPECT_LT(score, 100.0);
}

TEST(MapsComparison, CellsOutsideSharedDomainDoNotAffectScore) {
    const types::MapConfig origin_config =
        mapConfig(bounds(0.0, 30.0, 0.0, 10.0, 0.0, 10.0), 10.0);
    const types::MapConfig target_config =
        mapConfig(bounds(10.0, 40.0, 0.0, 10.0, 0.0, 10.0), 10.0);
    auto origin = Map3DImpl::createEmpty(origin_config, types::VoxelOccupancy::Empty);
    auto target = Map3DImpl::createEmpty(target_config, types::VoxelOccupancy::Empty);
    origin->set(pos(0.0, 0.0, 0.0), types::VoxelOccupancy::Occupied);
    target->set(pos(30.0, 0.0, 0.0), types::VoxelOccupancy::Occupied);

    EXPECT_DOUBLE_EQ(MapsComparison::compareSingle(*origin, *target), 100.0);
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

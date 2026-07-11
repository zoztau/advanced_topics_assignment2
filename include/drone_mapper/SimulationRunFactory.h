#pragma once

#include <drone_mapper/ISimulationRunFactory.h>

namespace drone_mapper {

class SimulationRunFactory final : public ISimulationRunFactory {
public:
    [[nodiscard]] std::unique_ptr<ISimulationRun>
    create(const types::SimulationConfigData& simulation,
           const types::MissionConfigData& mission,
           const types::DroneConfigData& drone,
           const types::LidarConfigData& lidar,
           const std::filesystem::path& output_path) override;
};

} // namespace drone_mapper

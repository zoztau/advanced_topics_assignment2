#pragma once

#include <drone_mapper/IDroneMovement.h>
#include <drone_mapper/MockGPS.h>

namespace drone_mapper {

class IMap3D;

// Optional implementation for the 
class MockMovement final : public IDroneMovement {
public:
    explicit MockMovement(MockGPS& gps);
    MockMovement(MockGPS& gps, const IMap3D& hidden_map, PhysicalLength drone_radius);

    types::MovementResult rotate(types::RotationDirection direction, HorizontalAngle angle) override;
    types::MovementResult advance(PhysicalLength distance) override;
    types::MovementResult elevate(PhysicalLength distance) override;

private:
    [[nodiscard]] bool pathIsClear(const Position3D& from, const Position3D& to) const;
    [[nodiscard]] bool sphereIsClear(const Position3D& center) const;

    MockGPS& gps_;
    const IMap3D* hidden_map_ = nullptr;
    PhysicalLength drone_radius_{};
};

} // namespace drone_mapper

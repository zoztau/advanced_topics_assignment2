#pragma once

#include <drone_mapper/IMappingAlgorithm.h>

#include <deque>
#include <optional>
#include <set>
#include <vector>
#include <cstddef>

namespace drone_mapper {

class MappingAlgorithmImpl final : public IMappingAlgorithm {
public:
    struct GridIndex {
        int x = 0;
        int y = 0;
        int z = 0;
    };

    struct Direction {
        int dx = 0;
        int dy = 0;
        int dz = 0;
    };

    using IMappingAlgorithm::IMappingAlgorithm;
    [[nodiscard]] types::MappingStepCommand nextStep(const types::DroneState& state,
                                                     const types::LidarScanResult* latest_scan) override;

private:
    [[nodiscard]] bool usesLeanScanPattern() const;
    [[nodiscard]] std::size_t maxScannedCells() const;
    [[nodiscard]] GridIndex gridFromPosition(const Position3D& position) const;
    [[nodiscard]] Position3D cellCenter(const GridIndex& index) const;
    [[nodiscard]] long long flatKey(const GridIndex& index) const;
    [[nodiscard]] bool inGrid(const GridIndex& index) const;
    [[nodiscard]] bool sameCell(const GridIndex& left, const GridIndex& right) const;
    [[nodiscard]] bool isKnownSafe(const GridIndex& index, const GridIndex& current) const;
    [[nodiscard]] bool isKnownSafePath(const GridIndex& from,
                                       const GridIndex& to,
                                       const GridIndex& current) const;
    [[nodiscard]] double coverageScore(const GridIndex& index) const;
    [[nodiscard]] double clearanceScore(const GridIndex& index) const;
    [[nodiscard]] double scanPotentialScore(const GridIndex& current,
                                            const Direction& direction) const;
    [[nodiscard]] bool isFrontier(const GridIndex& index) const;
    [[nodiscard]] std::vector<GridIndex> neighbors(const GridIndex& index) const;
    [[nodiscard]] std::optional<GridIndex> chooseLocalMove(const GridIndex& current,
                                                           Orientation heading) const;
    [[nodiscard]] std::vector<GridIndex> pathToQueuedBranchTarget(const GridIndex& current) const;
    [[nodiscard]] std::vector<GridIndex> pathToBestCoverageTarget(const GridIndex& current) const;
    [[nodiscard]] std::optional<types::MappingStepCommand> commandTowardTarget(
        const types::DroneState& state,
        const GridIndex& current,
        const GridIndex& target);
    [[nodiscard]] bool movementReachesTarget(const types::DroneState& state,
                                             const GridIndex& current,
                                             const GridIndex& target,
                                             const types::MappingStepCommand& command) const;
    [[nodiscard]] std::optional<Direction> takeFirstScanForCell(const GridIndex& current,
                                                                Orientation heading);
    void rememberBranchTargets(const GridIndex& current);
    void enqueueScansForCell(const GridIndex& current, Orientation heading);

    std::deque<Direction> pending_scans_{};
    std::deque<GridIndex> branch_targets_{};
    std::optional<GridIndex> pending_target_{};
    std::set<long long> scanned_cells_{};
    std::set<long long> visited_cells_{};
    std::set<long long> branch_target_keys_{};
};

} // namespace drone_mapper

#pragma once

#include <drone_mapper/IMappingAlgorithm.h>

#include <cstddef>
#include <deque>
#include <map>
#include <optional>
#include <set>
#include <tuple>
#include <utility>
#include <vector>

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
    struct SafetyAnalysis {
        bool hard_blocked = false;
        std::vector<GridIndex> unresolved{};

        [[nodiscard]] bool safe() const {
            return !hard_blocked && unresolved.empty();
        }
    };

    [[nodiscard]] bool usesLeanScanPattern() const;
    [[nodiscard]] std::size_t maxScannedCells() const;
    [[nodiscard]] types::MappingStepCommand finishedCommand() const;
    [[nodiscard]] GridIndex gridFromPosition(const Position3D& position) const;
    [[nodiscard]] Position3D cellCenter(const GridIndex& index) const;
    [[nodiscard]] long long flatKey(const GridIndex& index) const;
    [[nodiscard]] bool inGrid(const GridIndex& index) const;
    [[nodiscard]] bool sameCell(const GridIndex& left, const GridIndex& right) const;
    [[nodiscard]] SafetyAnalysis analyzeKnownSafety(const GridIndex& index,
                                                    const GridIndex& current,
                                                    bool collect_all_unresolved = true) const;
    [[nodiscard]] SafetyAnalysis analyzeKnownSafePath(const GridIndex& from,
                                                      const GridIndex& to,
                                                      const GridIndex& current,
                                                      bool collect_all_unresolved = true) const;
    [[nodiscard]] bool isKnownSafe(const GridIndex& index, const GridIndex& current) const;
    [[nodiscard]] bool isKnownSafePath(const GridIndex& from,
                                       const GridIndex& to,
                                       const GridIndex& current) const;
    [[nodiscard]] double coverageScore(const GridIndex& index) const;
    [[nodiscard]] double clearanceScore(const GridIndex& index) const;
    [[nodiscard]] double scanPotentialScore(const Position3D& origin,
                                            const Direction& direction) const;
    [[nodiscard]] double minimumUsefulScanPotential() const;
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
    [[nodiscard]] std::optional<Orientation> takeNextScanForCell(
        const GridIndex& current,
        const types::DroneState& state);
    void rememberBranchTargets(const GridIndex& current);

    std::deque<GridIndex> branch_targets_{};
    std::optional<GridIndex> pending_target_{};
    std::set<long long> scanned_cells_{};
    std::set<std::tuple<long long, int, int, int>> attempted_base_scans_{};
    std::set<std::tuple<long long, long long, long long, long long>> attempted_adaptive_rays_{};
    std::map<long long, std::size_t> adaptive_scan_counts_{};
    std::map<long long, std::size_t> adaptive_candidate_cursors_{};
    std::set<long long> visited_cells_{};
    std::set<long long> branch_target_keys_{};
};

} // namespace drone_mapper

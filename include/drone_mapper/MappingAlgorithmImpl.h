#pragma once
#include <drone_mapper/IMappingAlgorithm.h>
#include <array>
#include <optional>
#include <queue>
#include <set>

namespace drone_mapper {

class MappingAlgorithmImpl : public IMappingAlgorithm {
public:
    // התיקון: DroneConfigData עובר By Value
    MappingAlgorithmImpl(types::DroneConfigData drone_config, const IMap3D& output_map);

    types::MappingStepCommand nextStep(const types::DroneState& state,
                                       const types::LidarScanResult* latest_scan) override;

private:
    std::queue<Position3D> target_queue_;
    // Target currently being pursued: kept across nextStep() calls because
    // reaching it may take two commands (rotate to face it, then advance),
    // and DroneControlImpl only executes one movement command per step.
    std::optional<Position3D> pending_target_;
    // Grid cells (rounded to the map resolution) already enqueued or stood
    // on. A neighbor being Empty only means its own occupancy is known, not
    // that space beyond it has been explored, so the frontier must still
    // push through Empty cells -- this set is what stops that from turning
    // into an infinite back-and-forth between the same two cells.
    std::set<std::array<long long, 3>> visited_cells_;
    // Cells whose own neighbors have already been generated. Distinct from
    // visited_cells_ (which prevents re-enqueueing a cell as *someone else's*
    // candidate): this tracks whether we've expanded outward *from* a cell
    // once we actually stood on it, so every reached cell gets to spawn its
    // own frontier instead of only whichever cell happens to be current when
    // the queue drains.
    std::set<std::array<long long, 3>> expanded_cells_;
    bool is_finished_ = false;
};

} // namespace drone_mapper
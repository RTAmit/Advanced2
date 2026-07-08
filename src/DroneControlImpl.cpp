#include "drone_mapper/DroneControlImpl.h"
#include "drone_mapper/ScanResultToVoxels.h"

#include <utility>

namespace drone_mapper {

DroneControlImpl::DroneControlImpl(types::DroneConfigData drone,
                                   types::MissionConfigData mission,
                                   ILidar& lidar,
                                   IGPS& gps,
                                   IDroneMovement& movement,
                                   IMutableMap3D& output_map,
                                   IMappingAlgorithm& mapping_algorithm)
    : drone_(std::move(drone)),
      mission_(std::move(mission)),
      lidar_(lidar),
      gps_(gps),
      movement_(movement),
      output_map_(output_map),
      mapping_algorithm_(mapping_algorithm) {}

types::DroneStepResult DroneControlImpl::step() {
    types::DroneState current_state = state();

    if (step_index_ >= mission_.max_steps) {
        return types::DroneStepResult{types::DroneStepStatus::Error, "Max steps reached"};
    }

    types::MappingStepCommand next_move = mapping_algorithm_.nextStep(
        current_state, has_latest_scan_ ? &latest_scan_ : nullptr);

    // Movement must be performed before the scan, so the scan (if requested)
    // observes the drone's post-movement position and heading.
    if (next_move.movement.has_value()) {
        auto move_cmd = next_move.movement.value();
        if (move_cmd.type == types::MovementCommandType::Advance) {
            movement_.advance(move_cmd.distance);
        } else if (move_cmd.type == types::MovementCommandType::Rotate) {
            movement_.rotate(move_cmd.rotation, move_cmd.angle);
        } else if (move_cmd.type == types::MovementCommandType::Elevate) {
            movement_.elevate(move_cmd.distance);
        }
    }

    if (next_move.scan_orientation.has_value()) {
        types::DroneState post_move_state = state();
        latest_scan_ = lidar_.scan(next_move.scan_orientation.value());
        has_latest_scan_ = true;
        ScanResultToVoxels::applyToMap(output_map_, post_move_state.position, post_move_state.heading,
                                       latest_scan_, lidar_.config());
    } else {
        has_latest_scan_ = false;
    }

    step_index_++;

    if (next_move.status == types::AlgorithmStatus::Finished ||
        next_move.status == types::AlgorithmStatus::FinishedWithUnmappableVoxels) {
        return types::DroneStepResult{types::DroneStepStatus::Completed, "Finished"};
    }

    return types::DroneStepResult{types::DroneStepStatus::Continue, "Step completed"};
}

types::DroneState DroneControlImpl::state() const {
    return types::DroneState{gps_.position(), gps_.heading(), step_index_};
}

} // namespace drone_mapper

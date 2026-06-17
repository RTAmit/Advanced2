#include "drone_mapper/DroneControlImpl.h" // שים לב לכלול את הקובץ המקומי שלנו

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

    // 1. קריאה לסריקה מהליידאר
    types::LidarScanResult scan_result = lidar_.scan(current_state.heading); 

    // 2. קבלת הפקודה מהאלגוריתם
    types::MappingStepCommand next_move = mapping_algorithm_.nextStep(current_state, &scan_result);

    // 3. ביצוע התנועה אם נדרשת
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
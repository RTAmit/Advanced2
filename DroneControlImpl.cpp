#include "DroneControlImpl.h" // שים לב לכלול את הקובץ המקומי שלנו

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

    
    types::LidarScanResult scan_result = lidar_.performScan(); 
    types::MappingStepCommand next_move = mapping_algorithm_.nextStep(current_state, &scan_result);

    movement_.executeCommand(next_move);

    step_index_++;

    return types::DroneStepResult{types::DroneStepStatus::Continue, "Step completed"};
}

types::DroneState DroneControlImpl::state() const {
    return types::DroneState{gps_.position(), gps_.heading(), step_index_};
}

} // namespace drone_mapper
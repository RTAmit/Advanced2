#include <drone_mapper/MissionControlImpl.h>
#include <utility>

namespace drone_mapper {

MissionControlImpl::MissionControlImpl(types::MissionConfigData mission,
                                       types::DroneConfigData drone,
                                       const IMap3D& hidden_map,
                                       IMutableMap3D& output_map,
                                       IDroneControl& drone_control,
                                       std::filesystem::path output_map_file)
    : mission_(std::move(mission)),
      drone_(std::move(drone)),
      hidden_map_(hidden_map),
      output_map_(output_map),
      drone_control_(drone_control),
      output_map_file_(std::move(output_map_file)) {}

types::MissionRunResult MissionControlImpl::runMission() {
    types::MissionRunStatus final_status = types::MissionRunStatus::Completed; 
    std::optional<types::ErrorRef> error_ref = std::nullopt;
    std::size_t steps_taken = 0;

    for (std::size_t i = 0; i < mission_.max_steps; ++i) {
       
        types::DroneState current_state = drone_control_.state();
        auto pos = current_state.position;

        if (pos.x_cm < mission_.boundaries.x_boundary.min_cm || pos.x_cm > mission_.boundaries.x_boundary.max_cm ||
            pos.y_cm < mission_.boundaries.y_boundary.min_cm || pos.y_cm > mission_.boundaries.y_boundary.max_cm ||
            pos.height_cm < mission_.boundaries.height_boundary.min_cm || pos.height_cm > mission_.boundaries.height_boundary.max_cm) {
            
            final_status = types::MissionRunStatus::Error;
            error_ref = types::ErrorRef{"MISSION_BOUNDARY_INVALID", "Drone exited mission boundaries"};
            break; 
        }

        types::DroneStepResult step_result = drone_control_.step();
        steps_taken++;

        if (step_result.status == types::DroneStepStatus::Error) {
            final_status = types::MissionRunStatus::Error;
            error_ref = types::ErrorRef{"DRONE_ERROR", step_result.message};
            break;
        }
        
        if (step_result.status == types::DroneStepStatus::Finished) { break; }
    }

    output_map_.save(output_map_file_);

    return types::MissionRunResult{
        final_status,
        steps_taken,
        error_ref
    };
}

} // namespace drone_mapper
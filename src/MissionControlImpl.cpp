#include "MissionControlImpl.h"
#include <utility>

namespace drone_mapper {
    MissionControlImpl::MissionControlImpl(
    types::MissionConfigData mission,
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
      output_map_file_(std::move(output_map_file)) 
{
}
types::MissionRunResult MissionControlImpl::runMission() {
    types::MissionRunStatus final_status = types::MissionRunStatus::Completed;
    std::vector<types::ErrorRef> error_refs; 
    std::size_t steps_taken = 0;

    types::MappingBounds bounds = output_map_.getMapConfig().boundaries;

    for (std::size_t i = 0; i < mission_.max_steps; ++i) {
        types::DroneState current_state = drone_control_.state();
        auto pos = current_state.position;

        double px = pos.x.force_numerical_value_in(cm);
        double py = pos.y.force_numerical_value_in(cm);
        double pz = pos.z.force_numerical_value_in(cm);

        if (px < bounds.min_x.force_numerical_value_in(cm) || px > bounds.max_x.force_numerical_value_in(cm) ||
            py < bounds.min_y.force_numerical_value_in(cm) || py > bounds.max_y.force_numerical_value_in(cm) ||
            pz < bounds.min_height.force_numerical_value_in(cm) || pz > bounds.max_height.force_numerical_value_in(cm)) {
            
            final_status = types::MissionRunStatus::Error;
            error_refs.push_back(types::ErrorRef{"MISSION_BOUNDARY_INVALID", "Drone exited mission boundaries"});
            break;
        }

        types::DroneStepResult step_result = drone_control_.step();
        steps_taken++;

        if (step_result.status == types::DroneStepStatus::Error) {
            final_status = types::MissionRunStatus::Error;
            error_refs.push_back(types::ErrorRef{"DRONE_ERROR", step_result.message});
            break;
        }
        
        if (step_result.status == types::DroneStepStatus::Completed) { break; }
    }

    output_map_.save(output_map_file_);

    return types::MissionRunResult{
        final_status,
        steps_taken,
        error_refs
    };
}
} // namespace drone_mapper
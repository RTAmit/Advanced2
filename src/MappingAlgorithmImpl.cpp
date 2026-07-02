#include "drone_mapper/MissionControlImpl.h"
#include "drone_mapper/Units.h" // הוספת ייבוא היחידות
#include <utility>
#include <fstream>
#include <string>

// ייבוא מפורש כדי למנוע את שגיאת ה-Ambiguity של cm
using namespace mp_units::si::unit_symbols;

namespace drone_mapper {

// פונקציית עזר לכתיבה מיידית לקובץ לוג
void logErrorImmediately(const std::string& error_code, const std::string& message) {
    std::ofstream log_file("error_log.txt", std::ios_base::app);
    if (log_file.is_open()) {
        log_file << "[ERROR] Code: " << error_code << " | Message: " << message << "\n";
        log_file.flush(); // כתיבה מיידית לדיסק כפי שנדרש
        log_file.close();
    }
}

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
    bool mission_finished = false; 

    for (std::size_t i = 0; i < mission_.max_steps; ++i) {
        types::DroneState current_state = drone_control_.state();
        auto pos = current_state.position;

        // בדיקת גבולות משימה - שימוש בפונקציה המובנית שהיא גם קצרה וגם בטוחה יותר
        if (!output_map_.isInBounds(pos)) {
            std::string err_msg = "Drone exited mission boundaries";
            final_status = types::MissionRunStatus::Error;
            error_refs.push_back(types::ErrorRef{"MISSION_BOUNDARY_INVALID", err_msg});
            logErrorImmediately("MISSION_BOUNDARY_INVALID", err_msg);
            break;
        }

        types::DroneStepResult step_result = drone_control_.step();
        steps_taken++;

        if (step_result.status == types::DroneStepStatus::Error) {
            final_status = types::MissionRunStatus::Error;
            error_refs.push_back(types::ErrorRef{"DRONE_ERROR", step_result.message});
            logErrorImmediately("DRONE_ERROR", step_result.message);
            break;
        }

        if (step_result.status == types::DroneStepStatus::Completed) { 
            final_status = types::MissionRunStatus::Completed;
            mission_finished = true;
            break; 
        }
    }

    // טיפול במידה ונגמרו הצעדים והרחפן לא סיים
    if (!mission_finished && final_status != types::MissionRunStatus::Error) {
        final_status = types::MissionRunStatus::MaxSteps;
        logErrorImmediately("MISSION_MAX_STEPS", "Mission did not finish within allocated steps");
    }

    if (!output_map_file_.empty()) {
        output_map_.save(output_map_file_);
    }

    return types::MissionRunResult{final_status, steps_taken, error_refs};
}

} // namespace drone_mapper
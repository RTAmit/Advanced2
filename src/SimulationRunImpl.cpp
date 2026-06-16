#include <drone_mapper/SimulationRunImpl.h>
#include <drone_mapper/MapsComparison.h>
#include <stdexcept>
#include <utility>
#include <vector>

namespace drone_mapper {

SimulationRunImpl::SimulationRunImpl(std::unique_ptr<const IMap3D> hidden_map,
                                     std::unique_ptr<IMutableMap3D> output_map,
                                     std::unique_ptr<IGPS> gps,
                                     std::unique_ptr<IDroneMovement> movement,
                                     std::unique_ptr<ILidar> lidar,
                                     std::unique_ptr<IMappingAlgorithm> mapping_algorithm,
                                     std::unique_ptr<IDroneControl> drone_control,
                                     std::unique_ptr<IMissionControl> mission_control,
                                     types::SimulationConfigData simulation_config,
                                     types::MissionConfigData mission_config,
                                     std::filesystem::path output_map_file)
    : hidden_map_(std::move(hidden_map)),
      output_map_(std::move(output_map)),
      gps_(std::move(gps)),
      movement_(std::move(movement)),
      lidar_(std::move(lidar)),
      mapping_algorithm_(std::move(mapping_algorithm)),
      drone_control_(std::move(drone_control)),
      mission_control_(std::move(mission_control)),
      simulation_config_(std::move(simulation_config)),
      mission_config_(std::move(mission_config)),
      output_map_file_(std::move(output_map_file)) {}

types::SimulationResult SimulationRunImpl::run() {
    types::MissionRunResult mission_res = mission_control_->runMission();
    // 1. צור וקטור של shared_ptr
    std::vector<std::shared_ptr<IMap3D>> targets;

    // 2. הוסף את המפה (שים לב: output_map_ אצלך הוא unique_ptr, לכן צריך shared_ptr)
    // אם המפה שלך היא מסוג IMap3D, השתמש ב-shared_ptr
    targets.push_back(std::shared_ptr<IMap3D>(output_map_.get(), [](IMap3D*){})); 

    // 3. בצע את ההשוואה
    std::vector<double> scores = MapsComparison::compare(*hidden_map_, targets);
    double final_score = scores.empty() ? 0.0 : scores[0];

    types::SimulationResult result;
    result.simulation_config = simulation_config_;
    result.mission_config = mission_config_;
    
    // שימוש נכון בווקטור התוצאות החדש:
    result.mission_results.push_back(mission_res); 
    
    // שימוש בשם המשתנה המעודכן עבור הציון:
    result.mission_score = final_score;
    
    result.output_map_file = output_map_file_;
    
    // שמירת קונפיגורציית המפה לפלט:
    result.output_map_config = output_map_->getMapConfig();
    
    return result;
}

} // namespace drone_mapper
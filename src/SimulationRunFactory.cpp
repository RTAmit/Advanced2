#include "SimulationRunFactoryImpl.h"
#include "DroneControlImpl.h"
#include "Map3DImpl.h"
#include "MappingAlgorithmImpl.h"
#include "MissionControlImpl.h"
#include "MockGPS.h"
#include "MockLidar.h"
#include "MockMovement.h"
#include "SimulationRunImpl.h"
#include <vector>
#include <cstdint>
#include <memory>
#include <filesystem>

namespace drone_mapper {

std::unique_ptr<ISimulationRun>
SimulationRunFactoryImpl::create(const types::SimulationConfigData& simulation,
                                 const types::MissionConfigData& mission,
                                 const types::DroneConfigData& drone,
                                 const types::LidarConfigData& lidar,
                                 const std::filesystem::path& output_path) {
    
    // תיקון: שימוש בטיפוס המדויק של הספרייה עבור ה-Shape (שהוא std::vector<size_t>)
    NpyArray::shape_t dummy_shape = {1, 1, 1};
    std::vector<uint8_t> dummy_data = {0};
    
    // תיקון: טעינת המפה המקורית באמצעות יצירת אובייקט ריק וקריאה ל-LoadNPY
    auto hidden_npy = std::make_shared<NpyArray>();
    hidden_npy->LoadNPY(simulation.map_filename.string()); 
    auto hidden_map = std::make_unique<Map3DImpl>(hidden_npy); 

    // תיקון: העברת המצביע הגולמי dummy_data.data() לבנאי של NpyArray
    auto output_npy = std::make_shared<NpyArray>(dummy_shape, dummy_data.data(), false);
    auto output_map = std::make_unique<Map3DImpl>(output_npy);

    auto gps = std::make_unique<MockGPS>(
        simulation.initial_drone_position,
        Orientation{simulation.initial_angle, 0.0 * altitude_angle[deg]});
        
    auto movement = std::make_unique<MockMovement>(*gps);
    auto lidar_impl = std::make_unique<MockLidar>(lidar, *hidden_map, *gps);
    auto mapping_algorithm = std::make_unique<MappingAlgorithmImpl>(drone, *output_map);

    auto drone_control = std::make_unique<DroneControlImpl>(
        drone, mission, *lidar_impl, *gps, *movement, *output_map, *mapping_algorithm);

    const std::filesystem::path output_map_file = output_path / "map_output.npy";
    
    auto mission_control = std::make_unique<MissionControlImpl>(
        mission, drone, *hidden_map, *output_map, *drone_control, output_map_file);

    return std::make_unique<SimulationRunImpl>(
        std::move(hidden_map),
        std::move(output_map),
        std::move(gps),
        std::move(movement),
        std::move(lidar_impl),
        std::move(mapping_algorithm),
        std::move(drone_control),
        std::move(mission_control),
        simulation,
        mission,
        output_map_file
    );
}

} // namespace drone_mapper
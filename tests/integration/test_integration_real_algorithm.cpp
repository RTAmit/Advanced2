#include <gtest/gtest.h>
#include <drone_mapper/MissionControlImpl.h>
#include <drone_mapper/DroneControlImpl.h>
#include <drone_mapper/MappingAlgorithmImpl.h>
#include <drone_mapper/MockLidar.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockMovement.h>
#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MapsComparison.h>
#include <drone_mapper/Units.h>
#include <TinyNPY.h>
#include <memory>
#include <string>
#include <vector>

#include <mp-units/systems/si/units.h>

using namespace drone_mapper;
using namespace mp_units::si::unit_symbols;

void RunIntegrationOnMap(const std::string& map_path) {
    auto hidden_array = std::make_shared<NpyArray>();
    try {
        hidden_array->LoadNPY(map_path);
    } catch (...) {
        FAIL() << "Cannot load file: " << map_path;
    }
    
    types::MapConfig map_config;
    map_config.resolution = 10.0 * cm; 
    map_config.boundaries = types::MappingBounds{
        0.0 * cm, static_cast<double>(hidden_array->Shape()[0]) * 10.0 * cm,
        0.0 * cm, static_cast<double>(hidden_array->Shape()[1]) * 10.0 * cm,
        0.0 * cm, static_cast<double>(hidden_array->Shape()[2]) * 10.0 * cm
    };
    Map3DImpl hidden_map(hidden_array, map_config);

    auto output_array = std::make_shared<NpyArray>(hidden_array->Shape(), 1, 'u', false);
    output_array->Allocate();
    std::fill_n(output_array->Data<uint8_t>(), 
                output_array->Shape()[0] * output_array->Shape()[1] * output_array->Shape()[2], 
                255); 
    Map3DImpl output_map(output_array, map_config);

    types::MissionConfigData mission_config{2000, 10.0 * cm, 1}; 
    types::DroneConfigData drone_config;
    
    // z_min, z_max, d (ring spacing), fov_circles: fov_circles must be > 0 or
    // MockLidar::scan() returns no hits at all (see MockLidar.cpp), which is
    // what was silently zeroing out every mapping score below.
    types::LidarConfigData lidar_config{1.0 * cm, 500.0 * cm, 10.0 * cm, 3};

    Position3D start_pos{0.0 * cm, 0.0 * cm, 0.0 * cm};
    Orientation start_ori{0.0 * mp_units::non_si::degree};
    MockGPS gps(start_pos, start_ori);
    
    MockMovement movement(gps);
    MockLidar lidar(lidar_config, hidden_map, gps); 

    MappingAlgorithmImpl mapping_alg(drone_config, output_map); 
    
    DroneControlImpl drone_control(drone_config, mission_config, lidar, gps, movement, output_map, mapping_alg, lidar_config);
    MissionControlImpl mission_control(mission_config, drone_config, hidden_map, output_map, drone_control, ""); 

    types::MissionRunResult result = mission_control.runMission();
    
    MapsComparison comparer;
    std::vector<IMap3D*> targets = {&output_map};
    std::vector<double> scores = comparer.compare(hidden_map, targets);
    double score = scores.empty() ? 0.0 : scores[0]; 

    EXPECT_EQ(result.status, types::MissionRunStatus::Completed)
        << "Mission failed or got stuck in infinite loop on map: " << map_path;
    EXPECT_GE(score, 90.0)
        << "Mapping score too low (" << score << ") on map: " << map_path;
}

TEST(IntegrationRealAlgorithmTest, MapsFiveVoxelsPattern) {
    RunIntegrationOnMap("data_maps/five_voxels_y4_pattern.npy");
}

TEST(IntegrationRealAlgorithmTest, MapsSingleVoxelX2Y4Z2) {
    RunIntegrationOnMap("data_maps/single_voxel_x2_y4_z2.npy");
}

TEST(IntegrationRealAlgorithmTest, MapsSingleVoxelX4Y4Z4) {
    RunIntegrationOnMap("data_maps/single_voxel_x4_y4_z4.npy");
}

TEST(IntegrationRealAlgorithmTest, MapsTwoFullXPlanesPlusUpperPattern) {
    RunIntegrationOnMap("data_maps/two_full_x_planes_plus_upper_pattern.npy");
}
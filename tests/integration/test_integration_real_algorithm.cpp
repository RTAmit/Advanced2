#include <gtest/gtest.h>
#include <drone_mapper/MissionControlImpl.h>
#include <drone_mapper/DroneControlImpl.h>
#include <drone_mapper/MappingAlgorithmImpl.h>
#include <drone_mapper/MockLidar.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockMovement.h>
#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MapsComparison.h>
#include <TinyNPY.h>
#include <memory>
#include <string>

using namespace drone_mapper;
using namespace mp_units;

void RunIntegrationOnMap(const std::string& map_path) {
    auto hidden_array = std::make_shared<NpyArray>(map_path);
    
    types::MapConfig map_config;
    map_config.resolution = 10.0 * si::centi * si::metre; 
    map_config.boundaries = types::MappingBounds{
        0.0 * si::metre, static_cast<double>(hidden_array->Shape()[0]) * map_config.resolution,
        0.0 * si::metre, static_cast<double>(hidden_array->Shape()[1]) * map_config.resolution,
        0.0 * si::metre, static_cast<double>(hidden_array->Shape()[2]) * map_config.resolution
    };
    Map3DImpl hidden_map(hidden_array, map_config);

    auto output_array = std::make_shared<NpyArray>(hidden_array->Shape(), 1, 'u', false);
    output_array->Allocate();
    std::fill_n(output_array->Data<uint8_t>(), 
                output_array->Shape()[0] * output_array->Shape()[1] * output_array->Shape()[2], 
                255); 
    Map3DImpl output_map(output_array, map_config);

    types::MissionConfigData mission_config{2000, map_config.boundaries}; // מספיק צעדים למיפוי
    types::DroneConfigData drone_config;
    types::LidarConfigData lidar_config{5.0 * si::metre, 360.0 * isq::angle::degree, 0.1 * si::metre};

    MockLidar lidar(lidar_config, hidden_map);
    MockGPS gps(hidden_map);
    MockMovement movement(hidden_map);
    MappingAlgorithmImpl mapping_alg(drone_config, output_map); // ודא חתימת בנאי נכונה
    
    DroneControlImpl drone_control(drone_config, mission_config, lidar, gps, movement, output_map, mapping_alg);
    MissionControlImpl mission_control(mission_config, drone_config, hidden_map, output_map, drone_control, ""); // ללא קובץ יציאה בטסט

    types::MissionRunResult result = mission_control.runMission();
    
    MapsComparison comparer;
    double score = comparer.compare(hidden_map, output_map);

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
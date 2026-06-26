#include <gtest/gtest.h>
#include <drone_mapper/MockLidar.h>
#include <drone_mapper/Map3DImpl.h>
#include <TinyNPY.h>
#include <memory>
#include <vector>

using namespace drone_mapper;
using namespace mp_units;

TEST(MockLidarTest, DetectsObstacleAtEndOfBeam) {
    std::vector<unsigned long> shape = {10, 10, 10};
    auto npy_hidden = std::make_shared<NpyArray>(shape, 1, 'u', false);
    npy_hidden->Allocate();
    std::fill_n(npy_hidden->Data<uint8_t>(), 10 * 10 * 10, 0);
    
    types::MapConfig map_config;
    map_config.resolution = 1.0 * si::centi * si::metre;
    map_config.boundaries = types::MappingBounds{
        0.0 * si::metre, 0.1 * si::metre,
        0.0 * si::metre, 0.1 * si::metre,
        0.0 * si::metre, 0.1 * si::metre
    };
    Map3DImpl hidden_map(npy_hidden, map_config);
    
    Position3D obstacle_pos{0.09 * si::metre, 0.0 * si::metre, 0.0 * si::metre};
    hidden_map.set(obstacle_pos, types::VoxelOccupancy::Occupied);

    types::LidarConfigData lidar_config{0.1 * si::metre, 360.0 * isq::angle::degree, 0.01 * si::metre};
    MockLidar lidar(lidar_config, hidden_map);
    
    Position3D pos{0.0 * si::metre, 0.0 * si::metre, 0.0 * si::metre};
    types::Angle3D orientation{0.0 * isq::angle::degree, 0.0 * isq::angle::degree, 0.0 * isq::angle::degree};
    
    auto result = lidar.scan(pos, orientation);
    
    ASSERT_FALSE(result.distances.empty()) << "Lidar returned completely empty scan.";
    bool found_obstacle = false;
    for (auto dist : result.distances) {
        if (dist > 0.08 * si::metre && dist <= 0.1 * si::metre) {
            found_obstacle = true;
            break;
        }
    }
    
    EXPECT_TRUE(found_obstacle) << "Lidar failed to detect an obstacle at the far end of its z_max beam.";
}
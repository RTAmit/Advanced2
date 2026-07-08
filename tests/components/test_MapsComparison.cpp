#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <drone_mapper/MapsComparison.h>
#include <drone_mapper/IMap3D.h>
#include <vector>

using namespace drone_mapper;
using ::testing::Return;
using ::testing::_;

class MockMap3D : public IMap3D {
public:
    MOCK_METHOD(types::VoxelOccupancy, atVoxel, (const Position3D&), (const, override));
    MOCK_METHOD(types::MapConfig, getMapConfig, (), (const, override));
    MOCK_METHOD(bool, isInBounds, (const Position3D&), (const, override));
};

// Test 1: Identical maps should return a score of 100
TEST(MapsComparison, IdenticalMapsReturn100) {
    types::MapConfig cfg{};
    cfg.resolution = 10 * cm;
    cfg.boundaries.min_x = 0 * cm; cfg.boundaries.max_x = 50 * cm;
    cfg.boundaries.min_y = 0 * cm; cfg.boundaries.max_y = 50 * cm;
    cfg.boundaries.min_height = 0 * cm; cfg.boundaries.max_height = 50 * cm;

    MockMap3D map1;
    MockMap3D map2;

    EXPECT_CALL(map1, getMapConfig()).WillRepeatedly(Return(cfg));
    EXPECT_CALL(map2, getMapConfig()).WillRepeatedly(Return(cfg));

    EXPECT_CALL(map1, atVoxel(_)).WillRepeatedly(Return(types::VoxelOccupancy::Occupied));
    EXPECT_CALL(map2, atVoxel(_)).WillRepeatedly(Return(types::VoxelOccupancy::Occupied));

    EXPECT_CALL(map1, isInBounds(_)).WillRepeatedly(Return(true));
    EXPECT_CALL(map2, isInBounds(_)).WillRepeatedly(Return(true));

    std::vector<IMap3D*> targets = { &map2 };
    std::vector<double> scores = MapsComparison::compare(map1, targets);
    
    ASSERT_FALSE(scores.empty());
    EXPECT_DOUBLE_EQ(scores[0], 100.0);
}

// Bonus - Test 2: Maps with different resolutions
TEST(MapsComparison, SupportsDifferentResolutionsBonus) {
    // Define the original map with a resolution of 10 cm
    types::MapConfig cfg_origin{};
    cfg_origin.resolution = 10 * cm;
    cfg_origin.boundaries.min_x = 0 * cm; cfg_origin.boundaries.max_x = 20 * cm;
    cfg_origin.boundaries.min_y = 0 * cm; cfg_origin.boundaries.max_y = 20 * cm;
    cfg_origin.boundaries.min_height = 0 * cm; cfg_origin.boundaries.max_height = 20 * cm;

    // Destination map with a different resolution (20 cm)
    types::MapConfig cfg_target{};
    cfg_target.resolution = 20 * cm;
    cfg_target.boundaries = cfg_origin.boundaries; 
    MockMap3D map_origin;
    MockMap3D map_target;

    EXPECT_CALL(map_origin, getMapConfig()).WillRepeatedly(Return(cfg_origin));
    EXPECT_CALL(map_target, getMapConfig()).WillRepeatedly(Return(cfg_target));

    EXPECT_CALL(map_origin, atVoxel(_)).WillRepeatedly(Return(types::VoxelOccupancy::Occupied));
    EXPECT_CALL(map_target, atVoxel(_)).WillRepeatedly(Return(types::VoxelOccupancy::Occupied));

    EXPECT_CALL(map_origin, isInBounds(_)).WillRepeatedly(Return(true));
    EXPECT_CALL(map_target, isInBounds(_)).WillRepeatedly(Return(true));

    std::vector<IMap3D*> targets = { &map_target };
    
    std::vector<double> scores = MapsComparison::compare(map_origin, targets);
    
    ASSERT_FALSE(scores.empty());
    EXPECT_DOUBLE_EQ(scores[0], 100.0); 
}
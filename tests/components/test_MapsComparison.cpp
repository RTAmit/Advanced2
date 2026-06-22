#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <drone_mapper/MapsComparison.h>
#include <drone_mapper/IMap3D.h>
#include <vector>

using namespace drone_mapper;
using ::testing::Return;
using ::testing::_;

// התיקון: שינוי השם ל-atVoxel בדיוק כפי שמופיע בממשק IMap3D
class MockMap3D : public IMap3D {
public:
    MOCK_METHOD(types::VoxelOccupancy, atVoxel, (const Position3D&), (const, override));
    MOCK_METHOD(types::MapConfig, getMapConfig, (), (const, override));
};

TEST(MapsComparisonTest, IdenticalMapsReturn100) {
    types::MapConfig cfg{};
    cfg.resolution = 10 * cm;
    cfg.boundaries.min_x = 0 * cm; cfg.boundaries.max_x = 100 * cm;
    cfg.boundaries.min_y = 0 * cm; cfg.boundaries.max_y = 100 * cm;
    cfg.boundaries.min_height = 0 * cm; cfg.boundaries.max_height = 100 * cm;

    MockMap3D map1;
    MockMap3D map2;

    EXPECT_CALL(map1, getMapConfig()).WillRepeatedly(Return(cfg));
    EXPECT_CALL(map2, getMapConfig()).WillRepeatedly(Return(cfg));

    // התיקון: שימוש ב-atVoxel בפקודות ה-EXPECT_CALL
    EXPECT_CALL(map1, atVoxel(_)).WillRepeatedly(Return(types::VoxelOccupancy::Occupied));
    EXPECT_CALL(map2, atVoxel(_)).WillRepeatedly(Return(types::VoxelOccupancy::Occupied));

    std::vector<IMap3D*> targets = { &map2 };
    
    std::vector<double> scores = MapsComparison::compare(map1, targets);
    
    ASSERT_FALSE(scores.empty());
    EXPECT_DOUBLE_EQ(scores[0], 100.0);
}
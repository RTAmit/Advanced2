#include <gtest/gtest.h>
#include <drone_mapper/MapsComparison.h>
#include <drone_mapper/Map3DImpl.h>
#include <memory>
#include <vector>

using namespace drone_mapper;

TEST(MapsComparisonTest, IdenticalMapsReturn100) {
    std::vector<uint32_t> shape = {10, 10, 10};
    std::vector<uint8_t> data1(1000, 0);
    std::vector<uint8_t> data2(1000, 0);
    
    auto npy1 = std::make_shared<NpyArray>(shape, data1, false);
    auto npy2 = std::make_shared<NpyArray>(shape, data2, false);
    
    types::MapConfig cfg{};
    cfg.resolution = 10 * cm;
    cfg.boundaries.min_x = 0 * cm; cfg.boundaries.max_x = 100 * cm;
    cfg.boundaries.min_y = 0 * cm; cfg.boundaries.max_y = 100 * cm;
    cfg.boundaries.min_height = 0 * cm; cfg.boundaries.max_height = 100 * cm;

    Map3DImpl map1(npy1, cfg);
    Map3DImpl map2(npy2, cfg);
    
    Position3D pos{50 * cm, 50 * cm, 50 * cm};
    map1.set(pos, types::VoxelOccupancy::Occupied);
    map2.set(pos, types::VoxelOccupancy::Occupied);

    std::vector<IMap3D*> targets = { &map2 };
    std::vector<double> scores = MapsComparison::compare(map1, targets);
    
    ASSERT_FALSE(scores.empty());
    EXPECT_DOUBLE_EQ(scores[0], 100.0);
}
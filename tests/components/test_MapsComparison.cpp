#include <gtest/gtest.h>
#include "MapsComparison.h"
#include "Map3DImpl.h"

TEST(MapsComparisonTest, IdenticalMapsReturn100) {
    Map3DImpl map1(10);
    Map3DImpl map2(10);
    
    // Set a matching obstacle
    map1.setObstacle(50, 50, 50, true);
    map2.setObstacle(50, 50, 50, true);

    MapsComparison comparator;
    double score = comparator.compare(map1, map2);
    
    EXPECT_DOUBLE_EQ(score, 100.0);
}
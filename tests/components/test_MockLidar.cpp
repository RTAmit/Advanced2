#include <gtest/gtest.h>
#include <drone_mapper/MockLidar.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/Units.h>
#include <TinyNPY.h>
#include <limits>
#include <memory>
#include <vector>

using namespace drone_mapper;

namespace {

std::shared_ptr<NpyArray> makeEmptyArray(std::vector<unsigned long> shape) {
    auto arr = std::make_shared<NpyArray>(shape, 1, 'u', false);
    arr->Allocate();
    std::fill_n(arr->Data<uint8_t>(), shape[0] * shape[1] * shape[2], 0); // all Empty
    return arr;
}

types::MapConfig flatMapConfig() {
    types::MapConfig cfg;
    cfg.resolution = 10 * cm;
    cfg.boundaries = types::MappingBounds{0 * cm, 500 * cm, 0 * cm, 500 * cm, 0 * cm, 10 * cm};
    return cfg;
}

} // namespace

TEST(MockLidar, DetectsObstacleAtEndOfBeam) {
    auto array = makeEmptyArray({50, 50, 1});
    types::MapConfig cfg = flatMapConfig();
    Map3DImpl map(array, cfg);

    // Obstacle directly east (+X) of the origin at 100 cm.
    map.set(Position3D{100 * cm, 0 * cm, 5 * cm}, types::VoxelOccupancy::Occupied);

    MockGPS gps(Position3D{0 * cm, 0 * cm, 5 * cm}, Orientation{0 * deg, 0 * deg}, 10 * cm);
    types::LidarConfigData lidar_config{1 * cm, 500 * cm, 10 * cm, 1};
    MockLidar lidar(lidar_config, map, gps);

    // Scan straight ahead (0 relative to heading, which is already 0 = east).
    types::LidarScanResult result = lidar.scan(Orientation{0 * deg, 0 * deg});

    ASSERT_FALSE(result.empty());
    const auto& center_hit = result.front();
    EXPECT_NEAR(center_hit.distance.force_numerical_value_in(cm), 100.0, 1.0);
}

TEST(MockLidar, ReportsMissWhenNoObstacleWithinRange) {
    auto array = makeEmptyArray({50, 50, 1});
    Map3DImpl map(array, flatMapConfig());

    MockGPS gps(Position3D{0 * cm, 0 * cm, 5 * cm}, Orientation{0 * deg, 0 * deg}, 10 * cm);
    types::LidarConfigData lidar_config{1 * cm, 50 * cm, 10 * cm, 1};
    MockLidar lidar(lidar_config, map, gps);

    types::LidarScanResult result = lidar.scan(Orientation{0 * deg, 0 * deg});

    ASSERT_FALSE(result.empty());
    EXPECT_EQ(result.front().distance.force_numerical_value_in(cm), std::numeric_limits<double>::max());
}

TEST(MockLidar, ReturnsNoHitsWhenFovCirclesIsZero) {
    auto array = makeEmptyArray({50, 50, 1});
    Map3DImpl map(array, flatMapConfig());

    MockGPS gps(Position3D{0 * cm, 0 * cm, 5 * cm}, Orientation{0 * deg, 0 * deg}, 10 * cm);
    // fov_circles = 0 means no beams at all -- this must return an empty
    // scan rather than silently omitting only some beams, since a caller
    // that assumes at least a center beam exists would otherwise misbehave.
    types::LidarConfigData lidar_config{1 * cm, 500 * cm, 10 * cm, 0};
    MockLidar lidar(lidar_config, map, gps);

    types::LidarScanResult result = lidar.scan(Orientation{0 * deg, 0 * deg});

    EXPECT_TRUE(result.empty());
}

TEST(MockLidar, HitCloserThanZMinReportsZeroDistance) {
    auto array = makeEmptyArray({50, 50, 1});
    types::MapConfig cfg = flatMapConfig();
    Map3DImpl map(array, cfg);

    // Obstacle 15 cm away (a different voxel than the origin's own, given
    // the 10 cm resolution), closer than the sensor's 20 cm minimum range.
    map.set(Position3D{15 * cm, 0 * cm, 5 * cm}, types::VoxelOccupancy::Occupied);

    MockGPS gps(Position3D{0 * cm, 0 * cm, 5 * cm}, Orientation{0 * deg, 0 * deg}, 10 * cm);
    types::LidarConfigData lidar_config{20 * cm, 500 * cm, 10 * cm, 1};
    MockLidar lidar(lidar_config, map, gps);

    types::LidarScanResult result = lidar.scan(Orientation{0 * deg, 0 * deg});

    ASSERT_FALSE(result.empty());
    EXPECT_NEAR(result.front().distance.force_numerical_value_in(cm), 0.0, 1e-9);
}

TEST(MockLidar, ConfigReturnsTheConstructedConfig) {
    auto array = makeEmptyArray({50, 50, 1});
    Map3DImpl map(array, flatMapConfig());

    MockGPS gps(Position3D{0 * cm, 0 * cm, 5 * cm}, Orientation{0 * deg, 0 * deg}, 10 * cm);
    types::LidarConfigData lidar_config{20 * cm, 500 * cm, 10 * cm, 3};
    MockLidar lidar(lidar_config, map, gps);

    types::LidarConfigData returned = lidar.config();
    EXPECT_NEAR(returned.z_min.force_numerical_value_in(cm), 20.0, 1e-9);
    EXPECT_NEAR(returned.z_max.force_numerical_value_in(cm), 500.0, 1e-9);
    EXPECT_EQ(returned.fov_circles, 3u);
}

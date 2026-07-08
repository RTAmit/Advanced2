#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <drone_mapper/SimulationRunImpl.h>
#include <drone_mapper/IMissionControl.h>
#include <drone_mapper/IMappingAlgorithm.h>
#include <drone_mapper/ILidar.h>
#include <drone_mapper/IDroneControl.h>
#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockMovement.h>
#include <drone_mapper/Units.h>
#include <TinyNPY.h>
#include <memory>

using namespace drone_mapper;
using ::testing::NiceMock;
using ::testing::Return;

namespace {

class MockMissionControl : public IMissionControl {
public:
    MOCK_METHOD(types::MissionRunResult, runMission, (), (override));
};

class MockLidar : public ILidar {
public:
    MOCK_METHOD(types::LidarScanResult, scan, (Orientation), (const, override));
    MOCK_METHOD(types::LidarConfigData, config, (), (const, override));
};

class MockMappingAlgorithm : public IMappingAlgorithm {
public:
    using IMappingAlgorithm::IMappingAlgorithm;
    MOCK_METHOD(types::MappingStepCommand, nextStep, (const types::DroneState&, const types::LidarScanResult*), (override));
};

class MockDroneControl : public IDroneControl {
public:
    MOCK_METHOD(types::DroneStepResult, step, (), (override));
    MOCK_METHOD(types::DroneState, state, (), (const, override));
};

// Builds a small hidden/output map pair where every voxel is Occupied, so
// MapsComparison reports a known, deterministic score for the pairing.
std::shared_ptr<NpyArray> makeUniformArray(uint8_t value) {
    std::vector<unsigned long> shape = {2, 2, 2};
    auto arr = std::make_shared<NpyArray>(shape, 1, 'u', false);
    arr->Allocate();
    std::fill_n(arr->Data<uint8_t>(), 2 * 2 * 2, value);
    return arr;
}

types::MapConfig smallMapConfig() {
    types::MapConfig cfg;
    cfg.resolution = 10 * cm;
    cfg.boundaries = types::MappingBounds{0 * cm, 20 * cm, 0 * cm, 20 * cm, 0 * cm, 20 * cm};
    return cfg;
}

} // namespace

TEST(SimulationRun, ReturnsMissionResultScoreAndMetadataFromDependencies) {
    types::MapConfig cfg = smallMapConfig();

    // Hidden map: all Occupied (value 1). Output map: also all Occupied, so
    // the maps match perfectly and MapsComparison should report 100.
    auto hidden_map = std::make_unique<Map3DImpl>(makeUniformArray(1), cfg);
    auto output_map = std::make_unique<Map3DImpl>(makeUniformArray(1), cfg);

    auto gps = std::make_unique<MockGPS>(Position3D{}, Orientation{}, 10 * cm);
    auto movement = std::make_unique<MockMovement>(*gps);
    auto lidar = std::make_unique<NiceMock<MockLidar>>();
    auto mapping_algorithm = std::make_unique<MockMappingAlgorithm>(
        types::MissionConfigData{}, types::LidarConfigData{}, types::DroneConfigData{}, *output_map);
    auto drone_control = std::make_unique<NiceMock<MockDroneControl>>();

    auto mission_control = std::make_unique<MockMissionControl>();
    types::MissionRunResult mission_result{types::MissionRunStatus::Completed, 42, {}};
    EXPECT_CALL(*mission_control, runMission()).WillOnce(Return(mission_result));

    types::SimulationConfigData sim_cfg{};
    types::MissionConfigData mis_cfg{};

    SimulationRunImpl run(
        std::move(hidden_map), std::move(output_map), std::move(gps), std::move(movement),
        std::move(lidar), std::move(mapping_algorithm), std::move(drone_control),
        std::move(mission_control), sim_cfg, mis_cfg, "some_output_dir/map_output.npy",
        types::ResolutionRequestStatus::IgnoredTooSmall);

    types::SimulationResult result = run.run();

    ASSERT_EQ(result.mission_results.size(), 1u);
    EXPECT_EQ(result.mission_results[0].status, types::MissionRunStatus::Completed);
    EXPECT_EQ(result.mission_results[0].steps, 42u);
    EXPECT_DOUBLE_EQ(result.mission_score, 100.0);
    EXPECT_EQ(result.output_map_file, "some_output_dir/map_output.npy");
    EXPECT_EQ(result.resolution_request_status, types::ResolutionRequestStatus::IgnoredTooSmall);
}

TEST(SimulationRun, ScoresMismatchedMapsBelow100) {
    types::MapConfig cfg = smallMapConfig();

    auto hidden_map = std::make_unique<Map3DImpl>(makeUniformArray(1), cfg);  // all Occupied
    auto output_map = std::make_unique<Map3DImpl>(makeUniformArray(0), cfg); // all Empty

    auto gps = std::make_unique<MockGPS>(Position3D{}, Orientation{}, 10 * cm);
    auto movement = std::make_unique<MockMovement>(*gps);
    auto lidar = std::make_unique<NiceMock<MockLidar>>();
    auto mapping_algorithm = std::make_unique<MockMappingAlgorithm>(
        types::MissionConfigData{}, types::LidarConfigData{}, types::DroneConfigData{}, *output_map);
    auto drone_control = std::make_unique<NiceMock<MockDroneControl>>();

    auto mission_control = std::make_unique<MockMissionControl>();
    EXPECT_CALL(*mission_control, runMission())
        .WillOnce(Return(types::MissionRunResult{types::MissionRunStatus::Completed, 1, {}}));

    SimulationRunImpl run(
        std::move(hidden_map), std::move(output_map), std::move(gps), std::move(movement),
        std::move(lidar), std::move(mapping_algorithm), std::move(drone_control),
        std::move(mission_control), types::SimulationConfigData{}, types::MissionConfigData{}, "");

    types::SimulationResult result = run.run();

    EXPECT_LT(result.mission_score, 100.0);
}

// MockGPS and MockMovement have no dedicated component test file of their
// own, so their behavior is verified here per the assignment's instruction
// that this component test also covers the GPS and DroneMovement mocks.
TEST(SimulationRun, MockGPSStoresPositionAndHeading) {
    MockGPS gps(Position3D{1 * cm, 2 * cm, 3 * cm}, Orientation{10 * deg, 20 * deg}, 10 * cm);

    EXPECT_NEAR(gps.position().x.force_numerical_value_in(cm), 1.0, 1e-9);
    EXPECT_NEAR(gps.heading().horizontal.force_numerical_value_in(deg), 10.0, 1e-9);

    gps.setPosition(Position3D{5 * cm, 6 * cm, 7 * cm});
    gps.setHeading(Orientation{90 * deg, 0 * deg});

    EXPECT_NEAR(gps.position().y.force_numerical_value_in(cm), 6.0, 1e-9);
    EXPECT_NEAR(gps.heading().horizontal.force_numerical_value_in(deg), 90.0, 1e-9);
}

TEST(SimulationRun, MockMovementAdvancesAlongCurrentHeading) {
    MockGPS gps(Position3D{0 * cm, 0 * cm, 0 * cm}, Orientation{0 * deg, 0 * deg}, 10 * cm);
    MockMovement movement(gps);

    // Heading 0 (east): advancing should move purely along +X.
    movement.advance(10 * cm);
    EXPECT_NEAR(gps.position().x.force_numerical_value_in(cm), 10.0, 1e-9);
    EXPECT_NEAR(gps.position().y.force_numerical_value_in(cm), 0.0, 1e-9);
}

TEST(SimulationRun, MockMovementRotateChangesHeadingByDirection) {
    MockGPS gps(Position3D{}, Orientation{0 * deg, 0 * deg}, 10 * cm);
    MockMovement movement(gps);

    movement.rotate(types::RotationDirection::Left, 90 * deg);
    EXPECT_NEAR(gps.heading().horizontal.force_numerical_value_in(deg), 90.0, 1e-9);

    movement.rotate(types::RotationDirection::Right, 30 * deg);
    EXPECT_NEAR(gps.heading().horizontal.force_numerical_value_in(deg), 60.0, 1e-9);
}

TEST(SimulationRun, MockMovementElevateChangesHeightAndCanBeNegative) {
    MockGPS gps(Position3D{0 * cm, 0 * cm, 50 * cm}, Orientation{}, 10 * cm);
    MockMovement movement(gps);

    movement.elevate(10 * cm);
    EXPECT_NEAR(gps.position().z.force_numerical_value_in(cm), 60.0, 1e-9);

    movement.elevate(-25 * cm);
    EXPECT_NEAR(gps.position().z.force_numerical_value_in(cm), 35.0, 1e-9);
}

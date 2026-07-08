#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <drone_mapper/DroneControlImpl.h>
#include <drone_mapper/MissionControlImpl.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockLidar.h>
#include <drone_mapper/MockMovement.h>
#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/IMappingAlgorithm.h>
#include <drone_mapper/Units.h>
#include <TinyNPY.h>
#include <memory>

using namespace drone_mapper;
using ::testing::Return;

namespace {

// A scripted algorithm standing in for the real exploration logic: advance
// once, then report Finished. This exercises the full real wiring
// (DroneControlImpl -> MockMovement/MockGPS/MockLidar -> ScanResultToVoxels
// -> MissionControlImpl) along the "happy path" without depending on the
// real MappingAlgorithmImpl's exploration strategy.
class ScriptedMappingAlgorithm : public IMappingAlgorithm {
public:
    using IMappingAlgorithm::IMappingAlgorithm;

    types::MappingStepCommand nextStep(const types::DroneState&,
                                       const types::LidarScanResult*) override {
        types::MappingStepCommand cmd;
        types::MovementCommand move_cmd;

        if (call_count_ == 0) {
            move_cmd.type = types::MovementCommandType::Advance;
            move_cmd.distance = 10.0 * cm;
            cmd.movement = move_cmd;
            cmd.scan_orientation = Orientation{0.0 * deg, 0.0 * deg};
            cmd.status = types::AlgorithmStatus::Working;
        } else {
            move_cmd.type = types::MovementCommandType::Hover;
            cmd.movement = move_cmd;
            cmd.status = types::AlgorithmStatus::Finished;
        }
        ++call_count_;
        return cmd;
    }

private:
    int call_count_ = 0;
};

std::shared_ptr<NpyArray> makeEmptyArray(std::vector<unsigned long> shape) {
    auto arr = std::make_shared<NpyArray>(shape, 1, 'u', false);
    arr->Allocate();
    std::fill_n(arr->Data<uint8_t>(), shape[0] * shape[1] * shape[2], 0); // all Empty
    return arr;
}

} // namespace

TEST(Integration, MockAlgorithmDrivesFullWiringToCompletion) {
    types::MapConfig cfg;
    cfg.resolution = 10 * cm;
    cfg.boundaries = types::MappingBounds{0 * cm, 100 * cm, 0 * cm, 100 * cm, 0 * cm, 20 * cm};

    Map3DImpl hidden_map(makeEmptyArray({10, 10, 2}), cfg);
    Map3DImpl output_map(makeEmptyArray({10, 10, 2}), cfg);

    types::DroneConfigData drone_config{};
    types::MissionConfigData mission_config{};
    mission_config.max_steps = 10;

    types::LidarConfigData lidar_config{5 * cm, 100 * cm, 5 * cm, 1};

    MockGPS gps(Position3D{50 * cm, 50 * cm, 10 * cm}, Orientation{0 * deg, 0 * deg}, 10 * cm);
    MockMovement movement(gps);
    MockLidar lidar(lidar_config, hidden_map, gps);
    ScriptedMappingAlgorithm mapping_algorithm(mission_config, lidar_config, drone_config, output_map);

    DroneControlImpl drone_control(drone_config, mission_config, lidar, gps, movement, output_map,
                                   mapping_algorithm);
    MissionControlImpl mission_control(mission_config, drone_config, hidden_map, output_map,
                                       drone_control, "");

    types::MissionRunResult result = mission_control.runMission();

    EXPECT_EQ(result.status, types::MissionRunStatus::Completed);
    // The scripted Advance should have actually moved the drone forward.
    EXPECT_NEAR(gps.position().x.force_numerical_value_in(cm), 60.0, 1e-6);
}

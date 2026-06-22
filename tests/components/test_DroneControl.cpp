#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <drone_mapper/DroneControlImpl.h>
#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockLidar.h>
#include <drone_mapper/MockMovement.h>
#include <drone_mapper/Units.h>
#include <memory>

// השתמש ב-namespace כדי למנוע כפילויות
using namespace drone_mapper;
using ::testing::_;
using ::testing::Return;

class MockMappingAlgorithm : public IMappingAlgorithm {
public:
    // העברת הפרמטרים לבנאי מחלקת האב
    MockMappingAlgorithm(const types::DroneConfigData& drone_config, const IMap3D& output_map)
        : IMappingAlgorithm(drone_config, output_map) {}

    MOCK_METHOD(types::MappingStepCommand, nextStep, (const types::DroneState&, const types::LidarScanResult*), (override));
};

TEST(DroneControlTest, StepExecutesCorrectSequence) {
    types::DroneConfigData drone_cfg{};
    types::MissionConfigData mission_cfg{};
    mission_cfg.max_steps = 100;

    std::vector<unsigned long> shape = {1, 1, 1};
    auto npy = std::make_shared<NpyArray>(shape, 1, 'u', false);
    
    types::MapConfig map_cfg{};
    map_cfg.resolution = 10 * cm;
    
    Map3DImpl map(npy, map_cfg);

    // הגדרת המיקום בעזרת Position3D הסטנדרטי
    Position3D initial_pos{0 * cm, 0 * cm, 0 * cm};
    Orientation initial_ori{0 * deg, 0 * deg};
    
    MockGPS gps(initial_pos, initial_ori);
    MockMovement movement(gps);
    MockLidar lidar(types::LidarConfigData{}, map, gps);
    
    auto mockAlgo = std::make_unique<MockMappingAlgorithm>(drone_cfg, map);
    MockMappingAlgorithm* algoPtr = mockAlgo.get();

    DroneControlImpl droneControl(drone_cfg, mission_cfg, lidar, gps, movement, map, *mockAlgo);

    types::MappingStepCommand dummy_cmd;
    dummy_cmd.movement = types::MovementCommand{
        types::MovementCommandType::Advance, types::RotationDirection::Left, 0.0 * deg, 10.0 * cm
    };
    dummy_cmd.status = types::AlgorithmStatus::Working;

    EXPECT_CALL(*algoPtr, nextStep(_, _)).WillOnce(Return(dummy_cmd));

    types::DroneStepResult res = droneControl.step();
    EXPECT_EQ(res.status, types::DroneStepStatus::Continue);
}
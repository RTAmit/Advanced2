#include <gtest/gtest.h>
#include <drone_mapper/MissionControlImpl.h>
#include <drone_mapper/Map3DImpl.h>
#include <gmock/gmock.h>

using namespace drone_mapper;

class MockDroneControl : public IDroneControl {
public:
    MOCK_METHOD(types::DroneState, state, (), (const, override));
    MOCK_METHOD(types::DroneStepResult, step, (), (override));
};

TEST(MissionControlTest, RunMissionCompletesSuccessfully) {
    types::MissionConfigData mission_cfg{};
    mission_cfg.max_steps = 5;
    
    types::DroneConfigData drone_cfg{};
    
    std::vector<uint32_t> shape = {1, 1, 1};
    std::vector<uint8_t> data = {0};
    auto npy_hidden = std::make_shared<NpyArray>(shape, data, false);
    auto npy_output = std::make_shared<NpyArray>(shape, data, false);
    
    types::MapConfig map_cfg{};
    map_cfg.resolution = 10 * cm;
    map_cfg.boundaries.min_x = -100 * cm; map_cfg.boundaries.max_x = 100 * cm;
    map_cfg.boundaries.min_y = -100 * cm; map_cfg.boundaries.max_y = 100 * cm;
    map_cfg.boundaries.min_height = 0 * cm; map_cfg.boundaries.max_height = 100 * cm;
    
    Map3DImpl hidden_map(npy_hidden, map_cfg);
    Map3DImpl output_map(npy_output, map_cfg);
    
    MockDroneControl mock_drone;
    
    types::DroneState dummy_state{};
    dummy_state.position = Position3D{0 * cm, 0 * cm, 10 * cm};
    
    EXPECT_CALL(mock_drone, state()).WillRepeatedly(::testing::Return(dummy_state));
    EXPECT_CALL(mock_drone, step()).WillOnce(::testing::Return(types::DroneStepResult{types::DroneStepStatus::Completed, "Finished"}));
    
    MissionControlImpl mission(mission_cfg, drone_cfg, hidden_map, output_map, mock_drone, "test_output.npy");
    
    types::MissionRunResult res = mission.runMission();
    EXPECT_EQ(res.status, types::MissionRunStatus::Completed);
    EXPECT_EQ(res.steps_taken, 1);
}
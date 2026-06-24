#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <drone_mapper/MissionControlImpl.h>
#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/Types.h>
#include <drone_mapper/Units.h>
#include <memory>
#include <vector>

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
    
    // 1. הגדרת מערך מינימלי של 1x1x1 ווקסלים
    std::vector<unsigned long> shape = {1, 1, 1};
    
    auto npy_hidden = std::make_shared<NpyArray>(shape, 1, 'u', false);
    npy_hidden->Allocate(); // חובה! מקצה את הזיכרון בפועל למערך
    
    auto npy_output = std::make_shared<NpyArray>(shape, 1, 'u', false);
    npy_output->Allocate(); // חובה! מקצה את הזיכרון בפועל למערך
    
    // 2. התאמת גבולות המפה בדיוק לגודל המערך (רזולוציה של 10 ס"מ)
    types::MapConfig map_cfg{};
    map_cfg.resolution = 10 * cm;
    map_cfg.boundaries.min_x = 0 * cm; map_cfg.boundaries.max_x = 10 * cm;
    map_cfg.boundaries.min_y = 0 * cm; map_cfg.boundaries.max_y = 10 * cm;
    map_cfg.boundaries.min_height = 0 * cm; map_cfg.boundaries.max_height = 10 * cm;
    
    Map3DImpl hidden_map(npy_hidden, map_cfg);
    Map3DImpl output_map(npy_output, map_cfg);
    
    MockDroneControl mock_drone;
    
    // 3. תיקון קריטי: מיקום הרחפן (5,5,5) נמצא כעת באופן מוחלט בתוך הווקסל היחיד (0 עד 10)
    types::DroneState dummy_state{};
    dummy_state.position = Position3D{5 * cm, 5 * cm, 5 * cm};
    dummy_state.step_index = 0;
    
    EXPECT_CALL(mock_drone, state()).WillRepeatedly(::testing::Return(dummy_state));
    EXPECT_CALL(mock_drone, step()).WillOnce(::testing::Return(types::DroneStepResult{types::DroneStepStatus::Completed, "Finished"}));
    
    MissionControlImpl mission(mission_cfg, drone_cfg, hidden_map, output_map, mock_drone, "test_output.npy");
    
    types::MissionRunResult res = mission.runMission();
    
    // כעת הבדיקה תצפה לקבל Completed בהצלחה
    EXPECT_EQ(res.status, types::MissionRunStatus::Completed);
}
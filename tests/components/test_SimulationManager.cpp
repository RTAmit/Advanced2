#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <drone_mapper/SimulationManager.h>
#include <drone_mapper/ISimulationRunFactory.h>
#include <drone_mapper/ISimulationRun.h>
#include <memory>
#include <stdexcept>
#include <tuple>

using namespace drone_mapper;
using ::testing::_;
using ::testing::InvokeWithoutArgs;
using ::testing::Return;
using ::testing::Throw;

namespace {

class MockSimulationRun : public ISimulationRun {
public:
    MOCK_METHOD(types::SimulationResult, run, (), (override));
};

class MockSimulationRunFactory : public ISimulationRunFactory {
public:
    MOCK_METHOD(std::unique_ptr<ISimulationRun>, create,
               (const types::SimulationConfigData&, const types::MissionConfigData&,
                const types::DroneConfigData&, const types::LidarConfigData&,
                const std::filesystem::path&),
               (override));
};

types::SimulationCompositionData twoByTwoComposition() {
    types::SimulationCompositionData composition;
    composition.simulation_mission_groups = {
        std::tuple{types::SimulationConfigData{}, std::vector{types::MissionConfigData{}}},
        std::tuple{types::SimulationConfigData{}, std::vector{types::MissionConfigData{}}},
    };
    composition.drones = {types::DroneConfigData{}};
    composition.lidars = {types::LidarConfigData{}};
    return composition;
}

} // namespace

TEST(SimulationManager, NullFactoryThrows) {
    EXPECT_THROW(SimulationManager(nullptr), std::invalid_argument);
}

TEST(SimulationManager, RunsCartesianProductOfAllCombinations) {
    auto factory = std::make_unique<MockSimulationRunFactory>();

    // 2 simulations * 1 mission * 1 drone * 1 lidar = 2 combinations.
    EXPECT_CALL(*factory, create(_, _, _, _, _))
        .Times(2)
        .WillRepeatedly(InvokeWithoutArgs([]() {
            auto run = std::make_unique<MockSimulationRun>();
            EXPECT_CALL(*run, run()).WillOnce(Return(types::SimulationResult{}));
            return run;
        }));

    SimulationManager manager(std::move(factory));
    types::SimulationManagerReport report = manager.run(twoByTwoComposition(), "");

    EXPECT_EQ(report.runs.size(), 2u);
}

TEST(SimulationManager, FailedCombinationGetsErrorScoreAndDoesNotStopTheBatch) {
    auto factory = std::make_unique<MockSimulationRunFactory>();

    // First combination throws (e.g. unreadable map file); the second must
    // still run normally rather than the whole batch aborting.
    EXPECT_CALL(*factory, create(_, _, _, _, _))
        .WillOnce(Throw(std::runtime_error("map file not found")))
        .WillOnce(InvokeWithoutArgs([]() {
            auto run = std::make_unique<MockSimulationRun>();
            types::SimulationResult ok_result;
            ok_result.mission_score = 87.5;
            EXPECT_CALL(*run, run()).WillOnce(Return(ok_result));
            return run;
        }));

    SimulationManager manager(std::move(factory));
    types::SimulationManagerReport report = manager.run(twoByTwoComposition(), "");

    ASSERT_EQ(report.runs.size(), 2u);
    EXPECT_DOUBLE_EQ(report.runs[0].mission_score, -1.0);
    ASSERT_FALSE(report.runs[0].mission_results.empty());
    EXPECT_EQ(report.runs[0].mission_results.front().status, types::MissionRunStatus::Error);
    EXPECT_DOUBLE_EQ(report.runs[1].mission_score, 87.5);
}

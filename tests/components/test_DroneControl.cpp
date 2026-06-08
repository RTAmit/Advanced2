#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "DroneControlImpl.h"
#include "IMappingAlgorithm.h"
#include "ILidar.h"
#include "IGPS.h"
#include "IDroneMovement.h"

using ::testing::_;
using ::testing::Return;

// GMock classes
class MockMappingAlgorithm : public IMappingAlgorithm {
public:
    MOCK_METHOD(void, updateMap, (const Position3D&, const LidarScanResult&), (override));
    MOCK_METHOD(DroneCommand, calculateNextMove, (const Position3D&), (override));
};

class MockLidarSensor : public ILidar {
public:
    MOCK_METHOD(LidarScanResult, performScan, (), (override));
};

class MockGPSSensor : public IGPS {
public:
    MOCK_METHOD(Position3D, getPosition, (), (const, override));
};

class MockMovementController : public IDroneMovement {
public:
    MOCK_METHOD(void, executeCommand, (DroneCommand), (override));
};

TEST(DroneControlTest, StepExecutesCorrectSequence) {
    // 1. Setup the mocks
    auto mockMappingAlgo = std::make_unique<MockMappingAlgorithm>();
    MockMappingAlgorithm* algoPtr = mockMappingAlgo.get(); // Keep a raw pointer for setting expectations
    
    MockLidarSensor mockLidar;
    MockGPSSensor mockGPS;
    MockMovementController mockMovement;

    Position3D dummyPos{10, 10, 10};
    LidarScanResult dummyScan;
    
    // 2. Define Expectations (The order of calls is what we are testing)
    EXPECT_CALL(mockGPS, getPosition()).WillOnce(Return(dummyPos));
    EXPECT_CALL(mockLidar, performScan()).WillOnce(Return(dummyScan));
    EXPECT_CALL(*algoPtr, updateMap(_, _)).Times(1); // Expect map to be updated once
    EXPECT_CALL(*algoPtr, calculateNextMove(_)).WillOnce(Return(DroneCommand::Advance));
    EXPECT_CALL(mockMovement, executeCommand(DroneCommand::Advance)).Times(1);

    // 3. Initialize DroneControl
    DroneControlImpl droneControl(std::move(mockMappingAlgo), &mockLidar, &mockGPS, &mockMovement);
    
    // 4. Run the step
    droneControl.step();
}
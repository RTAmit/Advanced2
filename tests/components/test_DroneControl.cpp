#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <drone_mapper/DroneControlImpl.h>
#include <drone_mapper/Types.h>
#include <drone_mapper/Units.h>
#include <memory>

#include <mp-units/systems/si/units.h> 

using namespace drone_mapper;
using namespace mp_units::si::unit_symbols;
using ::testing::_;
using ::testing::Return;
using ::testing::NiceMock;

class MockIGPS : public IGPS {
public:
    MOCK_METHOD(Position3D, position, (), (const, override));
    MOCK_METHOD(Orientation, heading, (), (const, override));
};

class MockILidar : public ILidar {
public:
    MOCK_METHOD(types::LidarScanResult, scan, (Orientation scan_orientation), (const, override));
    
    // התיקון: חזרנו לחתימה המקורית והנכונה שהייתה לך
    MOCK_METHOD(types::LidarConfigData, config, (), (const, override)); 
};

class MockIDroneMovement : public IDroneMovement {
public:
    MOCK_METHOD(types::MovementResult, rotate, (types::RotationDirection direction, HorizontalAngle angle), (override));
    MOCK_METHOD(types::MovementResult, advance, (PhysicalLength distance), (override));
    MOCK_METHOD(types::MovementResult, elevate, (PhysicalLength distance), (override));
};

class MockIMutableMap3D : public IMutableMap3D {
public:
    MOCK_METHOD(types::MapConfig, getMapConfig, (), (const, override));
    MOCK_METHOD(bool, isInBounds, (const Position3D&), (const, override));
    MOCK_METHOD(types::VoxelOccupancy, atVoxel, (const Position3D&), (const, override));
    MOCK_METHOD(void, set, (const Position3D&, types::VoxelOccupancy), (override));
    MOCK_METHOD(void, save, (const std::filesystem::path&), (const, override));
};

class MockIMappingAlgorithm : public IMappingAlgorithm {
public:
    MockIMappingAlgorithm(const types::DroneConfigData& d, const IMap3D& m) : IMappingAlgorithm(d, m) {}
    MOCK_METHOD(types::MappingStepCommand, nextStep, (const types::DroneState&, const types::LidarScanResult*), (override));
};

TEST(DroneControlTest, StepExecutesCorrectSequenceAndReturnsStatus) {
    types::DroneConfigData drone_cfg{};
    types::MissionConfigData mission_cfg{};
    
    NiceMock<MockIGPS> mock_gps;
    NiceMock<MockILidar> mock_lidar;
    NiceMock<MockIDroneMovement> mock_movement;
    NiceMock<MockIMutableMap3D> mock_map;
    MockIMappingAlgorithm mock_algo(drone_cfg, mock_map);

    types::MovementResult success_result{true, ""};

    types::MappingStepCommand dummy_cmd;
    types::MovementCommand cmd;
    cmd.type = types::MovementCommandType::Advance;
    cmd.rotation = types::RotationDirection::Left;
    cmd.angle = 0.0 * mp_units::non_si::degree; 
    cmd.distance = 10.0 * cm; 
    dummy_cmd.movement = cmd;
    
    EXPECT_CALL(mock_algo, nextStep(_, _)).WillOnce(Return(dummy_cmd));
    
    EXPECT_CALL(mock_movement, advance(_))
        .Times(1)
        .WillOnce(Return(success_result));

    DroneControlImpl droneControl(drone_cfg, mission_cfg, mock_lidar, mock_gps, mock_movement, mock_map, mock_algo);

    types::DroneStepResult result = droneControl.step();

    EXPECT_EQ(result.status, types::DroneStepStatus::InProgress);
}
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
    MockIMappingAlgorithm(types::DroneConfigData d, const IMap3D& m) : IMappingAlgorithm(d, m) {}
    MOCK_METHOD(types::MappingStepCommand, nextStep, (const types::DroneState&, const types::LidarScanResult*), (override));
};

TEST(DroneControlTest, StepExecutesCorrectSequenceAndReturnsStatus) {
    types::DroneConfigData drone_cfg{};
    types::MissionConfigData mission_cfg{};
    mission_cfg.max_steps = 1; // default-constructed max_steps=0 would make step() bail out as "Max steps reached" before ever calling nextStep().
    
    NiceMock<MockIGPS> mock_gps;
    NiceMock<MockILidar> mock_lidar;
    NiceMock<MockIDroneMovement> mock_movement;
    NiceMock<MockIMutableMap3D> mock_map;
    
    // נקודה בתוך גבולות המפה (חשוב!)
    EXPECT_CALL(mock_map, isInBounds(_)).WillRepeatedly(Return(true));
    EXPECT_CALL(mock_gps, position()).WillRepeatedly(Return(Position3D{0*cm, 0*cm, 0*cm}));

    MockIMappingAlgorithm mock_algo(drone_cfg, mock_map);

    types::MappingStepCommand dummy_cmd;
    dummy_cmd.status = types::AlgorithmStatus::Working; 
    
    EXPECT_CALL(mock_algo, nextStep(_, _))
        .WillOnce(Return(dummy_cmd));
    
    DroneControlImpl droneControl(drone_cfg, mission_cfg, mock_lidar, mock_gps, mock_movement, mock_map, mock_algo);

    types::DroneStepResult result = droneControl.step();

    // הסטטוס צריך להיות Continue כפי שמוגדר ב-DroneTypes.h של הפרויקט
    EXPECT_EQ(result.status, types::DroneStepStatus::Continue);
}
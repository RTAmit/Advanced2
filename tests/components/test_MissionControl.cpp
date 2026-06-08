#include <gtest/gtest.h>
#include "MissionControlImpl.h" 
#include <fstream>
#include <filesystem>

class MissionControlTest : public ::testing::Test {
protected:
    std::string testConfigPath = "test_mission_config.yaml";

    void SetUp() override {
        // Create a temporary YAML file for testing
        std::ofstream out(testConfigPath);
        out << "mission_config:\n"
            << "  max_steps: 100\n"
            << "  boundaries:\n"
            << "    x_boundary: { min_cm: 0, max_cm: 100 }\n"
            << "    y_boundary: { min_cm: 0, max_cm: 100 }\n"
            << "    height_boundary: { min_cm: 0, max_cm: 100 }\n";
        out.close();
    }

    void TearDown() override {
        // Clean up the temporary file
        std::filesystem::remove(testConfigPath);
    }
};

TEST_F(MissionControlTest, MaxStepsLoadedCorrectly) {
    MissionControlImpl mission(testConfigPath);
    EXPECT_EQ(mission.getMaxSteps(), 100);
}

TEST_F(MissionControlTest, PositionWithinBoundaries) {
    MissionControlImpl mission(testConfigPath);
    Position3D validPos{50, 50, 50};
    EXPECT_TRUE(mission.isWithinBoundaries(validPos));
}

TEST_F(MissionControlTest, PositionOutsideBoundaries) {
    MissionControlImpl mission(testConfigPath);
    Position3D invalidPos{150, 50, 50}; // X is out of bounds
    EXPECT_FALSE(mission.isWithinBoundaries(invalidPos));
}
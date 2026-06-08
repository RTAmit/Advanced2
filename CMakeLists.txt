cmake_minimum_required(VERSION 3.16)
project(DroneMapperSimulation)

# Strict compilation flags as required by the assignment
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror -pedantic")

# Include current directory for all headers
include_directories(.)

# Find Google Test (GTest/GMock)
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

# Find yaml-cpp (Assuming it's installed on the system/Linux environment)
find_package(yaml-cpp REQUIRED)

# Common sources used by all targets (Flat directory for sources)
set(COMMON_SOURCES
    SimulationManager.cpp
    SimulationRunFactory.cpp
    SimulationRunImpl.cpp
    MissionControlImpl.cpp
    DroneControlImpl.cpp
    Map3DImpl.cpp
    MappingAlgorithmImpl.cpp
    MockGPS.cpp
    MockMovement.cpp
    MockLidar.cpp
)

# 1. Main Simulation Executable
add_executable(drone_mapper_simulation main.cpp ${COMMON_SOURCES})
target_link_libraries(drone_mapper_simulation yaml-cpp)

# 2. Standalone Maps Comparison Utility
add_executable(maps_comparison maps_comparison_main.cpp MapsComparison.cpp Map3DImpl.cpp)

# 3. Tests Executable (Updated to reflect your test folders!)
set(TEST_SOURCES
    tests/components/test_MissionControl.cpp
    tests/components/test_MapsComparison.cpp
    tests/components/test_DroneControl.cpp
    tests/integration/test_integration_real_algorithm.cpp
    # Add other test files here if you create them
)
add_executable(drone_mapper_simulation_test ${TEST_SOURCES} MapsComparison.cpp ${COMMON_SOURCES})
target_link_libraries(drone_mapper_simulation_test ${GTEST_LIBRARIES} gmock yaml-cpp pthread)
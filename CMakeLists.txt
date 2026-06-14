cmake_minimum_required(VERSION 3.16)
project(DroneMapperSimulation)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror -pedantic")


include_directories(.)

include_directories(biddai/ex_2_skeleton/include)

find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

find_package(yaml-cpp REQUIRED)

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

# 3. Tests Executable
set(TEST_SOURCES
    tests/components/test_MissionControl.cpp
    tests/components/test_MapsComparison.cpp
    tests/components/test_DroneControl.cpp
    tests/integration/test_integration_real_algorithm.cpp
)
add_executable(drone_mapper_simulation_test ${TEST_SOURCES} MapsComparison.cpp ${COMMON_SOURCES})

target_link_libraries(drone_mapper_simulation_test GTest::gtest GTest::gtest_main GTest::gmock yaml-cpp pthread)
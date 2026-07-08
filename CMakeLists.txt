cmake_minimum_required(VERSION 3.15)
project(drone_mapper LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)

# Required submission compile flags (Submission Guidelines #1): apply to every
# target in this project, not just the library, so main/test executables are
# held to the same standard.
add_compile_options(-Wall -Wextra -Werror -pedantic)

find_package(mp-units CONFIG REQUIRED)
find_package(tinynpy CONFIG REQUIRED)
find_package(GTest CONFIG REQUIRED) 
find_package(yaml-cpp REQUIRED) # new!

add_library(drone_mapper
    src/CompositionLoader.cpp
    src/DroneControlImpl.cpp
    src/MappingAlgorithmImpl.cpp
    src/MapsComparison.cpp
    src/MissionControlImpl.cpp
    src/Map3DImpl.cpp
    src/MockGPS.cpp
    src/MockLidar.cpp
    src/MockMovement.cpp
    src/ScanResultToVoxels.cpp
    src/SimulationManager.cpp
    src/SimulationRunFactoryImpl.cpp
    src/SimulationReportWriter.cpp
    src/ErrorLog.cpp
    src/SimulationRunImpl.cpp
)

target_compile_features(drone_mapper PUBLIC cxx_std_20)
target_include_directories(drone_mapper
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(drone_mapper
    PUBLIC
        mp-units::mp-units
        TinyNPY::TinyNPY
        yaml-cpp::yaml-cpp 
)

add_executable(drone_mapper_simulation src/drone_mapper_simulation_main.cpp)
target_link_libraries(drone_mapper_simulation PRIVATE drone_mapper)
#Compile the maps_comparison util
add_executable(maps_comparison src/maps_comparison_main.cpp)
target_link_libraries(maps_comparison PRIVATE drone_mapper)

# Compile the yaml example file
#add_executable(example_yml cpp_yaml_example/main.cpp)
#target_link_libraries(example_yml PRIVATE yaml-cpp::yaml-cpp)

add_executable(drone_mapper_simulation_test
    tests/components/test_DroneControl.cpp
    tests/components/test_MapsComparison.cpp
    tests/components/test_MissionControl.cpp
    tests/components/test_SimulationManager.cpp
    tests/components/test_SimulationRun.cpp
    tests/components/test_MappingAlgorithm.cpp
    tests/components/test_MockLidar.cpp
    tests/integration/test_integration_real_algorithm.cpp
    tests/integration/test_integration_mock_algorithm.cpp
)
target_link_libraries(drone_mapper_simulation_test PRIVATE drone_mapper GTest::gtest GTest::gtest_main GTest::gmock pthread)
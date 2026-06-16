cmake_minimum_required(VERSION 3.16)
project(DroneMapperSimulation LANGUAGES CXX)

# הגדרות קומפילציה קשיחות לפי דרישות המטלה
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror -pedantic")

# הגדרת נתיבים לחיפוש קובצי Header (הממשקים של השלד והקבצים שלנו)
include_directories(.)
include_directories(drone_mapper)

# ייבוא ספריות חיצוניות (vcpkg דואג להוריד ולספק אותן)
find_package(mp-units CONFIG REQUIRED)
find_package(tinynpy CONFIG REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

# רשימת קובצי המקור המשותפים (נתיבים מותאמים לתיקיית src ולתיקייה הראשית)
set(COMMON_SOURCES
    SimulationManager.cpp
    src/SimulationRunFactory.cpp
    src/SimulationRunImpl.cpp
    src/MissionControlImpl.cpp
    src/DroneControlImpl.cpp
    src/Map3DImpl.cpp
    src/MappingAlgorithmImpl.cpp
    src/MockGPS.cpp
    src/MockMovement.cpp
    src/MockLidar.cpp
)

# 1. יצירת קובץ ההרצה הראשי של הסימולציה
add_executable(drone_mapper_simulation main.cpp ${COMMON_SOURCES})
target_link_libraries(drone_mapper_simulation PRIVATE 
    yaml-cpp 
    mp-units::mp-units 
    TinyNPY::TinyNPY
)

# 2. יצירת תוכנית השוואת המפות (Utility)
# (במידה והקובץ maps_comparison_main.cpp נמצא בתיקייה הראשית ולא ב-src, יש למחוק את הקידומת /src משמו למטה)
add_executable(maps_comparison src/maps_comparison_main.cpp src/MapsComparison.cpp src/Map3DImpl.cpp)
target_link_libraries(maps_comparison PRIVATE 
    yaml-cpp 
    mp-units::mp-units 
    TinyNPY::TinyNPY
)

# 3. יצירת קובץ ההרצה לבדיקות האוטומטיות (Tests)
set(TEST_SOURCES
    tests/components/test_MissionControl.cpp
    tests/components/test_MapsComparison.cpp
    tests/components/test_DroneControl.cpp
    tests/integration/test_integration_real_algorithm.cpp
)
add_executable(drone_mapper_simulation_test ${TEST_SOURCES} src/MapsComparison.cpp ${COMMON_SOURCES})
target_link_libraries(drone_mapper_simulation_test PRIVATE 
    GTest::gtest 
    GTest::gtest_main 
    GTest::gmock 
    yaml-cpp 
    mp-units::mp-units 
    TinyNPY::TinyNPY 
    pthread
)
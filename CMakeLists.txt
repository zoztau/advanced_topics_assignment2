cmake_minimum_required(VERSION 3.15)
project(drone_mapper LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)


find_package(mp-units CONFIG REQUIRED)
find_package(tinynpy CONFIG REQUIRED)
find_package(yaml-cpp REQUIRED) # new!
find_package(GTest CONFIG REQUIRED)

add_library(drone_mapper
    src/config/ConfigLoader.cpp
    src/comparison/MapsComparison.cpp
    src/drone/DroneControlImpl.cpp
    src/geometry/VoxelGeometry.h
    src/gps/MockGPS.cpp
    src/lidar/MockLidar.cpp
    src/map/Map3DImpl.cpp
    src/movement/MockMovement.cpp
    src/mission/MappingAlgorithmImpl.cpp
    src/mission/MissionControlImpl.cpp
    src/scan_result_to_voxels/ScanResultToVoxels.cpp
    src/simulation/SimulationManager.cpp
    src/simulation/SimulationRunFactory.cpp
    src/simulation/SimulationRunImpl.cpp
)

target_compile_features(drone_mapper PUBLIC cxx_std_20)
target_compile_options(drone_mapper
    PRIVATE
        -Wall
        -Wextra
        -Werror
        -pedantic
)
target_include_directories(drone_mapper
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(drone_mapper
    PUBLIC
        mp-units::mp-units
        TinyNPY::TinyNPY
        yaml-cpp::yaml-cpp
)

add_executable(drone_mapper_simulation src/main/main.cpp)
target_link_libraries(drone_mapper_simulation PRIVATE drone_mapper)
#Compile the maps_comparison util
add_executable(maps_comparison src/comparison/maps_comparison_main.cpp)
target_link_libraries(maps_comparison PRIVATE drone_mapper)

add_executable(drone_mapper_simulation_test
    tests/components/component_tests.cpp
    tests/integration/integration_tests.cpp
)
target_compile_features(drone_mapper_simulation_test PRIVATE cxx_std_20)
target_compile_options(drone_mapper_simulation_test
    PRIVATE
        -Wall
        -Wextra
        -Werror
        -pedantic
)
target_link_libraries(drone_mapper_simulation_test
    PRIVATE
        drone_mapper
        GTest::gtest_main
        GTest::gmock
)

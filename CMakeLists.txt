cmake_minimum_required(VERSION 3.16)

project(robot_simulator
    VERSION 0.4.0
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

configure_file(
    include/robot_simulation/version.hpp.in
    ${CMAKE_CURRENT_BINARY_DIR}/generated/robot_simulation/version.hpp
    @ONLY
)

find_package(nlohmann_json CONFIG REQUIRED)

add_library(robot_simulator
    src/robot.cpp
    src/environment.cpp
    src/simulation.cpp
    src/geometry.cpp
    src/obstacle.cpp
    src/distance_sensor.cpp
    src/avoid_obstacles_controller.cpp
    src/distance_sensors_front_setup.cpp
    src/scene_config.cpp
    src/simulation_runner.cpp
)

target_link_libraries(robot_simulator
    PUBLIC
        nlohmann_json::nlohmann_json
)

target_include_directories(robot_simulator
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_BINARY_DIR}/generated
)

add_executable(robot_simulator_app
    app/main.cpp
)

target_link_libraries(robot_simulator_app
    PRIVATE
        robot_simulator
)

include(CTest)

enable_testing()

if(BUILD_TESTING)
    include(FetchContent)

    FetchContent_Declare(
        googletest
        URL
        https://github.com/google/googletest/archive/refs/tags/v1.14.0.tar.gz
    )

    FetchContent_MakeAvailable(googletest)

add_executable(robot_simulator_tests
    tests/geometry_tests.cpp
    tests/robot_tests.cpp
    tests/obstacle_tests.cpp
    tests/environment_tests.cpp
    tests/distance_sensor_tests.cpp
    tests/simulation_tests.cpp
    tests/avoid_obstacles_controller_tests.cpp
    tests/distance_sensors_front_setup_tests.cpp
    tests/scene_config_tests.cpp
    tests/simulation_runner_tests.cpp)

target_link_libraries(robot_simulator_tests
    PRIVATE
        robot_simulator
        GTest::gtest_main
)

include(FetchContent)

set(HTTPLIB_INSTALL OFF CACHE BOOL "" FORCE)
set(HTTPLIB_TEST OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
    cpp_httplib
    GIT_REPOSITORY https://github.com/yhirose/cpp-httplib.git
    GIT_TAG v0.60.1
)

FetchContent_MakeAvailable(cpp_httplib)

add_executable(robot_simulator_server
    app/server_main.cpp
)

target_link_libraries(robot_simulator_server
    PRIVATE
    robot_simulator
    httplib::httplib
)

include(GoogleTest)
gtest_discover_tests(robot_simulator_tests)
endif()

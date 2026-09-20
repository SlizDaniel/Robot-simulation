cmake_minimum_required(VERSION 3.16)

project(robot_simulator
    VERSION 0.1
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(robot_simulator
    src/robot.cpp
    src/environment.cpp
    src/simulation.cpp
    src/geometry.cpp
    src/obstacle.cpp
)

target_include_directories(robot_simulator
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
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
    tests/obstacle_tests.cpp)

target_link_libraries(robot_simulator_tests
    PRIVATE
        robot_simulator
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(robot_simulator_tests)
endif()


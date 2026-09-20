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
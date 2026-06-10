cmake_minimum_required(VERSION 3.16)
project(robot_control VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_library(robot_control_lib
    src/differential_drive_sim.cpp
    src/control_loop.cpp
)
target_include_directories(robot_control_lib
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(robot_control src/main.cpp)
target_link_libraries(robot_control PRIVATE robot_control_lib pthread)

enable_testing()
add_executable(robot_control_tests tests/test_differential_drive.cpp)
target_link_libraries(robot_control_tests PRIVATE robot_control_lib)
add_test(NAME differential_drive COMMAND robot_control_tests)

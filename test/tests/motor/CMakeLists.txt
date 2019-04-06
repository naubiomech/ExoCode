cmake_minimum_required(VERSION 2.6)
project(test_motor)
add_compile_options(-std=c++11)

file(GLOB SOURCES "*.cpp")

find_package(biomech_exo)

include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})
include_directories(${mock_port_analog_input_SOURCE_DIR})

################################
# Unit Tests
################################
# Link test executable against gtest & gtest_main

add_executable(test_motor testMotor.cpp)
target_link_libraries(test_motor gtest gtest_main biomech_exo mock_ports_analog_input)
gtest_add_tests(test_motor "" AUTO)

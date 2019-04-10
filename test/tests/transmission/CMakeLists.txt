cmake_minimum_required(VERSION 2.6)
project(test_transmission)
add_compile_options(-std=c++11)

file(GLOB SOURCES "*.cpp")

find_package(biomech_exo)

include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})
include_directories(${mock_port_analog_input_SOURCE_DIR})

################################
# Unit Tests
################################
# Link test executable against gtest & gtest_main

add_executable(${PROJECT_NAME}_base testBaseTransmission.hpp)
target_link_libraries(${PROJECT_NAME}_base gtest gtest_main mock_transmission mock_transceiver)
gtest_add_tests(${PROJECT_NAME}_base "" AUTO)

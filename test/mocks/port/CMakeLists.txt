cmake_minimum_required(VERSION 2.6)
project(mock_ports)
add_compile_options(-std=c++11)

find_package(biomech_exo)

################################
# Unit Tests
################################
# Link test executable against gtest & gtest_main

add_library(${PROJECT_NAME}_analog_input STATIC )
target_link_libraries(${PROJECT_NAME}_analog_input gtest gmock biomech_exo)
target_include_directories(${PROJECT_NAME}_analog_input PUBLIC . )

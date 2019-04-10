cmake_minimum_required(VERSION 2.6)
project(mock_transmission)
add_compile_options(-std=c++11)

find_package(biomech_exo)

################################
# Unit Tests
################################
# Link test executable against gtest & gtest_main

add_library(${PROJECT_NAME} STATIC mockBasicTransmission.hpp)
target_link_libraries(${PROJECT_NAME} gtest gmock biomech_exo)
target_include_directories(${PROJECT_NAME} PUBLIC . )

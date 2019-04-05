cmake_minimum_required(VERSION 2.6)
project(test_utils)
add_compile_options(-std=c++11)

file(GLOB SOURCES "*.cpp")

find_package(biomech_exo)
find_package(GTest)
include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

################################
# Unit Tests
################################
# Link test executable against gtest & gtest_main

# add_library(test_utils_range STATIC ${SOURCES})

add_executable(test_utils_range testRange.cpp)
target_link_libraries(test_utils_range gtest gtest_main biomech_exo)
add_executable(test_utils_max testMax.cpp)
target_link_libraries(test_utils_max gtest gtest_main biomech_exo)
gtest_add_tests(test_utils_range "" AUTO)
gtest_add_tests(test_utils_max "" AUTO)

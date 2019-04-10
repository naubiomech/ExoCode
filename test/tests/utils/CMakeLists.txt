cmake_minimum_required(VERSION 2.6)
project(test_utils)
add_compile_options(-std=c++11)

file(GLOB SOURCES "*.cpp")

find_package(biomech_exo)

include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

################################
# Unit Tests
################################
# Link test executable against gtest & gtest_main

add_executable(${PROJECT_NAME}_range testRange.cpp)
target_link_libraries(${PROJECT_NAME}_range gtest gtest_main biomech_exo)
gtest_add_tests(${PROJECT_NAME}_range "" AUTO)

add_executable(${PROJECT_NAME}_max testMax.cpp)
target_link_libraries(${PROJECT_NAME}_max gtest gtest_main biomech_exo)
gtest_add_tests(${PROJECT_NAME}_max "" AUTO)

add_executable(${PROJECT_NAME}_min testMin.cpp)
target_link_libraries(${PROJECT_NAME}_min gtest gtest_main biomech_exo)
gtest_add_tests(${PROJECT_NAME}_min "" AUTO)

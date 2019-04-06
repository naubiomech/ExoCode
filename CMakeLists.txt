cmake_minimum_required (VERSION 2.8.3)
project (biomech_exo)
add_compile_options(-std=c++11)


include_directories(./)
file(GLOB SOURCES "*.cpp")

set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)

add_library(biomech_exo STATIC ${SOURCES})
add_executable(exo ${SOURCES})

option(BUILD_TESTS "Enable testing" OFF)
if (BUILD_TESTS)
  include(CTest)
  add_subdirectory(lib/gtest)
  enable_testing()
  add_subdirectory(test)
endif()

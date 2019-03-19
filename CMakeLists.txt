cmake_minimum_required (VERSION 2.8.3)
project (biomech_exo)

include_directories(./)
#set(SOURCES main.cpp)
file(GLOB SOURCES "*.cpp")

set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)

add_library(biomech_exo STATIC ${SOURCES})

# add_executable(biomech_exo ${SOURCES})

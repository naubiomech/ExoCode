cmake_minimum_required (VERSION 2.8.3)
project (biomech_exo)
add_compile_options(-std=c++11)


include_directories(./)
file(GLOB SOURCES "*.cpp")

set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)

add_library(biomech_exo STATIC ${SOURCES})
add_executable(exo ${SOURCES})
target_link_libraries(biomech_exo PUBLIC coverage_config)
target_link_libraries(exo PUBLIC coverage_config)

option(BUILD_TESTS "Enable testing" OFF)
if (BUILD_TESTS)
  include(CTest)
  add_subdirectory(lib/gtest)
  enable_testing()
  add_subdirectory(test)
endif()

add_library(coverage_config INTERFACE)
option(CODE_COVERAGE "Enable coverage reporting" OFF)
if(CODE_COVERAGE AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  # Add required flags (GCC & LLVM/Clang)
  target_compile_options(coverage_config INTERFACE
	-O0        # no optimization
	-g         # generate debug info
	--coverage # sets all required flags
	)
  target_link_libraries(coverage_config INTERFACE --coverage)
endif()


add_library(coverage_config INTERFACE)
option(CODE_COVERAGE "Enable coverage reporting" OFF)
if(CODE_COVERAGE AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  # Add required flags (GCC & LLVM/Clang)
  include(.travis/CodeCoverage.cmake)
  target_compile_options(coverage_config INTERFACE
	-O0        # no optimization
	-g         # generate debug info
	--coverage # sets all required flags
	)
  target_link_libraries(coverage_config INTERFACE --coverage)

  include(.travis/CodeCoverage.cmake)
  APPEND_COVERAGE_COMPILER_FLAGS()
  set(COVERAGE_LCOV_EXCLUDES '/usr/include/*' 'test/*' 'lib/*')
  SETUP_TARGET_FOR_COVERAGE_LCOV_CLEAN(
	NAME coverage
	EXECUTABLE ctest -j ${n_cores} # Executable in PROJECT_BINARY_DIR
	DEPENDENCIES
	test_utils_min
	test_utils_range
	test_utils_max)
endif()

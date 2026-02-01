#
# src/CMakeLists.txt
#
cmake_minimum_required(VERSION 3.14)
project(core LANGUAGES CXX)
# option(BUILD_DOC "Build doxygen documentation" OFF)
option(BUILD_EXAMPLES "Build Examples" ON)
option(BUILD_UNIT_TESTS "Build Unit tests" ON)

# Path to helper functions
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
include(CmakeUtils)

# Add modules
add_subdirectory(core)
add_subdirectory(utils)
add_subdirectory(vehicleModel)









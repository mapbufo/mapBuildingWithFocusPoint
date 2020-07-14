cmake_minimum_required(VERSION 3.5)

PROJECT(mapBuildingWithFocusPoint)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")


set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )

## Dependencies
include_directories( ${PROJECT_SOURCE_DIR}/include )
add_subdirectory( src )
add_subdirectory( main )

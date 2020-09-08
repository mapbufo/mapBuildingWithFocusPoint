cmake_minimum_required(VERSION 2.8)

project(ReedsSheppPath)

 

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
set(CMAKE_BUILD_TYPE "Release") 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")


include_directories( ${PROJECT_SOURCE_DIR} )
add_executable(main main.cpp)


 

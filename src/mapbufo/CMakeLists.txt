cmake_minimum_required(VERSION 3.5)
project(mapbufo)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  roslib
  nav_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mapbufo
   CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs nav_msgs
#  DEPENDS system_lib
)

include_directories(
  ${PROJECT_SOURCE_DIR}/include/mapbufo
  ${catkin_INCLUDE_DIRS}
)

# add_library( lidar_sim src/simulator/lidar_sim.cpp )
add_library( map_sim src/simulator/map_simulator.cpp)
add_library( map src/map.cpp)
add_library( librobot src/robot.cpp)
add_library( path_planning src/path_planning.cpp)

# main function
add_executable(${PROJECT_NAME}_node main/main.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
#  lidar_sim
  map_sim
  map
  librobot
  path_planning
)

# send the coordinate-transform
add_executable(transform_support src/transform_support.cpp)

target_link_libraries(transform_support
  ${catkin_LIBRARIES}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_mapbufo.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

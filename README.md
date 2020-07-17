# mapBuildingWithFocusPoint


## Introduction
This project attempts to accomplish a new Lite-Slam, which is especially suitable for the device with low computational ability, or in a new environment with no map or simple map(even with semantic map). The kernel idea is that the input data will be filtered with priority so that the device could make decision in real-time.


## Requirements
- Develop-Environment: Ubuntu ****, ROS
- Software: Gazebo, Rviz
- Hardware: Turtlebot, Lidar
- Compiling: CMake

## Description
In this project a robot with a mounted lidar is prepared. Lidar has range e.g. 120 degree*5m. The scan-data from lidar should be collected in the message form `pointscloud`. From the scan-data only the points in `interested area` will be fully processed without filtering, e.g. +- 30 degree from mid, the other points will filtered. A `global map with low resolution` will be created and updated by the scan-data. The further navigation will be accomplished with the created map.

There is a detailed example. Assuming that we have a robot with lidar at the down-left corner in a room. There are many obstacles in this room like sofa and boxes. This task of this robot is reaching a target at the up-right corner of this room. This robot will start with a global empty map, which could be updated by the scan-data from lidar. A path to the target will be calculated with this global map. But the movement of the robot will be decided by the scan-data. The path could be re-calculated with updated global-map when needed.

## Develop-Plan
### Milestone 1
- [x] create a map-simulator by using txt to create a simple map with obstacles(10 x 10).
- [x] create a lidar-simulator, which scans the created map and return the scan-data.
- [x] create a robot class for controlling the movement of robot
- [x] create a map class for the created map by the scan-data, this map could be updated.
- [x] choose a algorithm to calculate the path from robot to target.
- [x] final-task: the robot could successfully reach the target.
- [ ] summary for milestone 1:

### Milestone 2
- [ ] create a simple scenario in Gazebo.
- [ ] write a subscriber to subscribe the lidar scan ScanData.
- [ ] create a global Map.
- [ ] call path_planning.
- [ ] move the robot
- [ ] summary for milestone 2:

## Some features nice to have
- GTest
- use `semantic-map`
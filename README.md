# mapBuildingWithFocusPoint


## Introduction
The aim of this project is to design a Lite-SLAM suitable for devices with limited computing power. This system also supports the incorporation of simple maps (e.g. semantic map) as extra information to enhance its performance in navigation. 

The kernel idea of this project is the filtering and prioritization of the input data, reducing the workload of the device in computation without losing too much information.


## Requirements
- Develop-Environment: Ubuntu ****, ROS
- Software: Gazebo, Rviz
- Hardware: Turtlebot, Lidar
- Compiling: CMake

## Description
This Lite-SLAM and Navigation system can be roughly divided into three parts: the collection and filtering of the input data (e.g. scan data), the creation and modification of a global map with low resolution and the navigation with scan data and the created map. 

Collection and filtering: The scan-data from lidar is stored in the form of `point cloud`. Here, only the points in the `interested area` will be fully processed without (or with minimum) filtering; the rest of the scan points are filtered with different criteria. For example, if the heading of the lidar is set to 0 degree and the interested range is set from -30 degree to 30 degree, all the scan points inside this area will be taken into consideration for further steps; those outside this range will be filtered and only a small amount of them will be kept and used in later processes.

Creation and modification of a low-resolution global map: A global map is created and updated with every filtered scan-data. This map is used for a rough path planning in navigation.

Navigation: The navigation is accomplished based on the low resolution map and the received scan data. The global map provides a rough direction for the robot, while the current scan data gives a detailed description of the visible environment for an exact path planning.

Here is a detailed example: assume that we have a robot with lidar at the down-left corner in a room. There are many obstacles in this room such as sofas and boxes. The task of this robot is reaching a target at the up-right corner of this room. This robot will start with an empty global map, updated by the scan-data from the lidar step by step. A suggested path to the target will be calculated based on this global map. The robot will choose the best path according to the suggested path and the current scan-data. The suggested path from the global map can be re-calculated with the updated global map when needed (e.g. when the planned path is blocked by an unseen obstacle later on).

## Develop-Plan
### Milestone 1
- [x] create a map-simulator using txt to create a simple map with obstacles (10 x 10).
- [x] create a lidar-simulator, which scans the created map and return the scan-data.
- [x] create a robot class for controlling the movement of the robot.
- [x] create a map class for the created map by the scan-data. This map should be able to be updated.
- [x] choose an algorithm to calculate the path from the robot to target.
- [x] final-task: the robot should successfully reach the target.
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
- use CNN to the object-detection and semantic-segmentation
- consider the moving object, use maybe KF

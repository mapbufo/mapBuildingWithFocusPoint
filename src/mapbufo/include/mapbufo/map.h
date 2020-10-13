#pragma once

#include <iostream>

#include "common.h"
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>

class Map
{
public:
  // used to create global map
  Map(ros::NodeHandle &nh);

  // used to create local map with specified parameter
  Map(ros::NodeHandle &nh, float resolution, int width, int height);

  ~Map(){};

  /**
   * @brief reset all the map data to -1
   */
  void resetMapData();

  /**
   * @brief overload the [] function to simplify getting the cell status
   * @param input pos: position of this cell
   * @return CellOccupied: status of this cell
   */
  CellOccupied operator[](Point2D point) { return getCell(point); };

  /**
   * @brief return the 4 quadrants maps as vector
   * @return std::vector, nav_msgs::OccupancyGrid map
   */
  std::vector<nav_msgs::OccupancyGrid> &getMap() { return maps_; }

  /**
   * @brief get the cell status
   * @param input x, y: position of this cell
   * @return CellOccupied: status of this cell
   */
  CellOccupied getCell(int x, int y);

  /**
   * @brief get the cell status
   * @param input pos: position of this cell
   * @return CellOccupied: status of this cell
   */
  CellOccupied getCell(Point2D pos);

  /**
   * @brief update the occupied probability of the cell
   * @param input x, y: position in each quadrant
   * @param input value: update probability
   * @param input global: if this is local map, then call the local update
   */
  status::status update(int x, int y, int update_value, bool global);

  /**
   * @brief update the occupied probability of the cell
   * @param input pos: position in each quadrant
   * @param input value: update probability
   * @param input global: if this is local map, then call the local update
   */
  status::status update(Point2D pos, int update_value, bool global);

  /**
   * @brief update the probability of the points, which are in the line between the
   * robot and scan point
   * @param input x0, y0: position of robot
   * @param input x1, y1: position of target point
   * @param input update_value: the should be added probability
   * @param input global: if this is local map, then call the local update
   */
  status::status updateWithScanPoint(float x0, float y0, float x1, float y1,
                                     int update_value, bool global);

  /**
   * @brief update the probability of the points with input unordered_map
   * @param input robot_pos: current robot position(float)
   * @param input curr_scan: current scan points
   *              with the cooresponding update value
   */
  status::status updateWithScanPoints(Point2DWithFloat robot_pos,
                                      ScanPointsFloatWithUpdateValue curr_scan);

  /**
   * @brief at first clear all the data from last cycle, then update the probability
   * of the points in the local map with input unordered_map, all the points
   * outside the local map should not be calculated
   * @param input robot_pos: current robot position(float)
   * @param input curr_scan: current scan points
   *              with the cooresponding update value
   */
  status::status updateLocalMapWithScanPoints(Point2DWithFloat robot_pos,
                                              ScanPointsFloatWithUpdateValue curr_scan);

private:
  // save the map data
  std::vector<nav_msgs::OccupancyGrid> maps_;

  // if the probability of this grid is bigger than occupied_bound, then this
  // cell will be treated as occupied
  int occupied_bound_;

  // if the probability of this grid is smaller than empty_bound,
  // then this cell will be treated as empty
  int empty_bound_;

  // in order to get the rosparam
  ros::NodeHandle nh_;

  // max range of the liader
  float lidar_range_;

private:
  /**
   * @brief transform the position from global into a single quadrant
   * @param input x, y: position in global
   * @param output x_qua, int y_qua: position in single quadrant
   * @param output qua: number of quadrant
   * @return CellOccupied: status of this cell
   */
  void transformPositionIntoQuadrant(int x, int y, int &x_qua, int &y_qua,
                                     int &qua);

  /**
   * @brief get the cell from the selected single quadrant
   * @param input x, y: position in each quadrant
   * @param input qua: number of quadrant
   * @return CellOccupied: status of this cell
   */
  CellOccupied getCellInSingleQuadrant(int x, int y, int qua);

  /**
   * @brief update the occupied probability of the cell from the selected single
   * quadrant
   * @param input x, y: position in each quadrant
   * @param input qua: number of quadrant
   * @param input value: update probability
   * @param input global: if this is local map, then do not extend the map size
   * @return CellOccupied: status of this cell
   */
  status::status updateCellInSingleQuadrant(int x, int y, int qua, int value, bool global);

  /**
   * @brief get the needed parameter from file /param/parameter.yaml
   */
  void getParam();
};

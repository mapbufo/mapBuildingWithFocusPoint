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
   * reset all the map data to -1
   */
  void ResetMapData();

  /**
   * overload the [] function to simplify getting the cell status
   * @param input Point2D pos: position of this cell
   * @return CellOccupied: status of this cell
   */
  CellOccupied operator[](Point2D point) { return GetCell(point); };

  /**
   * return the 4 quadrants maps as vector
   * @return std::vector, nav_msgs::OccupancyGrid map
   */
  std::vector<nav_msgs::OccupancyGrid> &GetMap() { return maps_; }

  /**
   * get the cell status
   * @param input int x, int y: position of this cell
   * @return CellOccupied: status of this cell
   */
  CellOccupied GetCell(int x, int y);

  /**
   * get the cell status
   * @param input Point2D pos: position of this cell
   * @return CellOccupied: status of this cell
   */
  CellOccupied GetCell(Point2D pos);

  /**
   * update the occupied probability of the cell
   * @param input int x, int y: position in each quadrant
   * @param input int value: update probability
   * @param input bool global: if this is local map, then call the local update
   */
  status::status Update(int x, int y, int update_value, bool global);

  /**
   * update the occupied probability of the cell
   * @param input Point2D pos: position in each quadrant
   * @param input int value: update probability
   * @param input bool global: if this is local map, then call the local update
   */
  status::status Update(Point2D pos, int update_value, bool global);

  /**
   * update the probability of the points, which are in the line between the
   * robot and scan point
   * @param input int x0, int y0: position of robot
   * @param input int x1, int y1: position of target point
   * @param input int update_value: the should be added probability
   * @param input bool global: if this is local map, then call the local update
   */
  status::status UpdateWithScanPoint(float x0, float y0, float x1, float y1,
                                     int update_value, bool global);

  /**
   * update the probability of the points with input unordered_map
   * @param input Point2DWithFloat robot_pos: current robot position(float)
   * @param input ScanPointsFloatWithUpdateValue curr_scan: current scan points
   *              with the cooresponding update value
   */
  status::status UpdateWithScanPoints(Point2DWithFloat robot_pos,
                                      ScanPointsFloatWithUpdateValue curr_scan);

  /**
   * at first clear all the data from last cycle, then update the probability
   * of the points in the local map with input unordered_map, all the points
   * outside the local map should not be calculated
   * @param input Point2DWithFloat robot_pos: current robot position(float)
   * @param input ScanPointsFloatWithUpdateValue curr_scan: current scan points
   *              with the cooresponding update value
   */
  status::status UpdateLocalMapWithScanPoints(Point2DWithFloat robot_pos,
                                              ScanPointsFloatWithUpdateValue curr_scan);

private:
  /**
   * save the map data
   */
  std::vector<nav_msgs::OccupancyGrid> maps_;

  /**
   * if the probability of this grid is bigger than occupied_bound,
   * then this cell will be treated as occupied
   */
  int occupied_bound_;

  /**
   * if the probability of this grid is smaller than empty_bound,
   * then this cell will be treated as empty
   */
  int empty_bound_;

  /**
   * in order to get the rosparam
   */
  ros::NodeHandle nh_;

  /**
   * max range of the liader
   */
  float lidar_range_;

private:
  /**
   * transform the position from global into a single quadrant
   * @param input int x, int y: position in global
   * @param output int x_qua, int y_qua: position in single quadrant
   * @param output qua: number of quadrant
   * @return CellOccupied: status of this cell
   */
  void TransformPositionIntoQuadrant(int x, int y, int &x_qua, int &y_qua,
                                     int &qua);

  /**
   * get the cell from the selected single quadrant
   * @param input int x, int y: position in each quadrant
   * @param input int qua: number of quadrant
   * @return CellOccupied: status of this cell
   */
  CellOccupied GetCellInSingleQuadrant(int x, int y, int qua);

  /**
   * update the occupied probability of the cell from the selected single
   * quadrant
   * @param input int x, int y: position in each quadrant
   * @param input int qua: number of quadrant
   * @param input int value: update probability
   * @param input bool global: if this is local map, then do not extend the map size
   * @return CellOccupied: status of this cell
   */
  status::status UpdateCellInSingleQuadrant(int x, int y, int qua, int value, bool global);

  /**
   * get the needed parameter from file /param/parameter.yaml
   */
  void GetParam();
};

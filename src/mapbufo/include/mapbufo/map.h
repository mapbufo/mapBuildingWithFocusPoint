#pragma once

#include <iostream>

#include "common.h"
#include <nav_msgs/OccupancyGrid.h>
#include <tf2/LinearMath/Quaternion.h>

class Map {
public:
  // used to create global map
  Map();
  ~Map(){};

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
   */
  status::status Update(int x, int y, int update_value);

  /**
   * update the occupied probability of the cell
   * @param input Point2D pos: position in each quadrant
   * @param input int value: update probability
   */
  status::status Update(Point2D pos, int update_value);

  /**
   * update the probability of the points, which are in the line between the
   * robot and scan point
   * @param input int x0, int y0: position of robot
   * @param input int x1, int y1: position of target point
   * @param input int update_value: the should be added probability
   */
  status::status UpdateWithScanPoint(float x0, float y0, float x1, float y1,
                                     int update_value);

  /**
   * get the line-points with gradient between 0 and 1
   * @param input int x0, int y0: position of robot
   * @param input int x1, int y1: position of target point
   * @return vector<Point2D>: all the passed points in the line
   */
  std::vector<Point2D> GetLineLow(int x0, int y0, int x1, int y1);

  /**
   * get the line-points with gradient greater than 1
   * @param input int x0, int y0: position of robot
   * @param input int x1, int y1: position of target point
   * @return vector<Point2D>: all the passed points in the line
   */
  std::vector<Point2D> GetLineHigh(int x0, int y0, int x1, int y1);

  /**
   * get the line-points between robot and target point
   * @param input int x0, int y0: position of robot
   * @param input int x1, int y1: position of target point
   * @return vector<Point2D>: all the passed points in the line
   */
  std::vector<Point2D> GetLine(int x0, int y0, int x1, int y1);

public:
  /**
   * each time the probability should be changed by this value, plus or minus
   * if the input data is filtered, then the update_value is lower
   */
  const int update_value_filtered = 6;

  /**
   * if the input data is original without filtering, then the update_value is
   * is higher
   */
  const int update_value_full = 30;

private:
  /**
   * save the map data
   */
  std::vector<nav_msgs::OccupancyGrid> maps_;

  /**
   * if the probability of this grid is bigger than occupied_bound,
   * then this cell will be treated as occupied
   */
  int occupied_bound = 75;

  /**
   * if the probability of this grid is smaller than empty_bound,
   * then this cell will be treated as empty
   */
  int empty_bound = 25;

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
   * @return CellOccupied: status of this cell
   */
  status::status UpdateCellInSingleQuadrant(int x, int y, int qua, int value);

  /**
   * transform the position from float real position to int grid position
   * @param input float x, float y: position of robot(real data)
   * @param return  Point2D: position of robot(grid position)
   */
  Point2D TransformIndex(float x, float y);
};

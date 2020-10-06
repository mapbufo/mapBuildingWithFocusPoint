#pragma once
#include <math.h>

#include <algorithm>
#include <iostream>
#include <memory>
#include <vector>

#include "common.h"
#include "map.h"
class Robot
{
private:
  int x_;  // x coordinate of the lidar
  int y_;  // y coordinate of the lidar

  float heading_;  // lidar direction

public:
  Robot()
  {
    x_ = y_ = 0;
    heading_ = 0.0;
  }

  Robot(int x, int y, float heading)
  {
    x_ = x;
    y_ = y;
    heading_ = heading;
  }
  ~Robot()
  {
  }

  // *************** state info ***************
  inline int getPosX()
  {
    return x_;
  }
  inline int getPosY()
  {
    return y_;
  }

  inline float getHeading()
  {
    return heading_;
  }

  inline void setPosX(int x)
  {
    x_ = x;
  }
  inline void setPosY(int y)
  {
    y_ = y;
  }
  inline void setHeading(float heading)
  {
    heading_ = heading;
  }

  // *************** estimate next step ***************
  /**
  * find the best next position based on the input scan data
  * @param scan: scan data with position and status (blocked, free, etc.)
  * @param candidate_pt: a point to be checked if it's the best target point
  * @param best_pos: best target point
  * @param best_heading: best heading, from current robot position to the best target point
  * @param only_empty: if only unoccupied scan cells are considered
  */
  void findBestPos(ScanData scan, Point2D candidate_pt, Point2D &best_pos, float &best_heading, bool only_empty);

  /**
    * estimate the next point for the robot
    * @param scan: scan data with position and status
    * @param end_pos: end pos in the map
    * @param max_dist: maximum lidar range
    * @param call_help: if no best point found, call external help
    * @return: next point for the robot to go
    */
  Point2D estimateNextStep(ScanData scan_data, Point2D end_pos, float max_dist, bool &call_help);

  // *************** move ***************
  /**
    * update robot states according to best next point
    * @param next_pos: next point position
    */
  void move(Point2D next_pos);
};

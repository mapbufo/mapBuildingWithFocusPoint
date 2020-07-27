#ifndef ROBOT_H
#define ROBOT_H

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
  int getPosX()
  {
    return x_;
  }
  int getPosY()
  {
    return y_;
  }

  float getHeading()
  {
    return heading_;
  }

  void setPosX(int x)
  {
    x_ = x;
  }
  void setPosY(int y)
  {
    y_ = y;
  }
  void setHeading(float heading)
  {
    heading_ = heading;
  }

  // *************** estimate next step ***************
  void findBestPos(ScanData scan, Point2D candidate_pt, Point2D &best_pos, float &best_heading, bool only_empty);

  Point2D estimateNextStep(ScanData scan_data, Point2D end_pos, float max_dist, bool &call_help);
  // *************** move ***************
  // update robot states according to next pos
  void move(Point2D next_pos);
};

#endif  // ROBOT_H

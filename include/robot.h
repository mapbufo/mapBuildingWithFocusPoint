#ifndef ROBOT_H
#define ROBOT_H

#include "map.h"
#include <algorithm>
#include <iostream>
#include <math.h>
#include <memory>
#include <vector>
class Robot {
private:
  int x_; // x coordinate of the lidar
  int y_; // y coordinate of the lidar

  float heading_; // lidar direction:
  // heading = 0 if the center of the lidar is aligned with the x+ axis;
  // heading = PI/2 if aligned with the y+ axis
  // heading = -PI/2 if aligned with the y- axis
  // heading = +-PI if aligned with the x- axis

public:
  Robot() {
    x_ = y_ = 0;
    heading_ = 0.0;
  }

  Robot(int x, int y, float heading) {
    x_ = x;
    y_ = y;
    heading_ = heading;
  }
  ~Robot() {}

  // *************** state info ***************
  int getPosX() { return x_; }
  int getPosY() { return y_; }

  float getHeading() { return heading_; }

  void setPosX(int x) { x_ = x; }
  void setPosY(int y) { y_ = y; }
  void setHeading(float heading) { heading_ = heading; }

  // *************** estimate next step ***************
  // 1) if no obstacle in sight (in the direction of target) then go straight in
  // that direction with step size = max_dist_;
  // 2) else if obstacle exists, then check the scan data from heading to +- max
  // angle and see if there're gaps elsewhere. In this case, turn to that
  // direction with step_size_ = 0.

  // return value: 0 - just direction; 1 - next pos estimated
  std::pair<int, int>
  estimateNextStep(Map scan_data, std::pair<int, int> end_pos, float max_dist);

  // *************** move ***************
  // update robot states according to next pos
  void move(std::pair<int, int> next_pos);
};

#endif // ROBOT_H

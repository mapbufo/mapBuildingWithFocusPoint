#ifndef ROBOT_H
#define ROBOT_H

#include <math.h>
#include <algorithm>
#include <iostream>
#include <memory>
#include <vector>
#include "map.h"
class Robot {
 private:
  int x_;            // x coordinate of the lidar
  int y_;            // y coordinate of the lidar
  float max_dist_;   // maximum detection range
  float max_angle_;  // maximum detection angle in radius
  float heading_;    // lidar direction:
  // heading = 0 if the center of the lidar is aligned with the x+ axis;
  // heading = PI/2 if aligned with the y+ axis
  // heading = -PI/2 if aligned with the y- axis
  // heading = +-PI if aligned with the x- axis

 public:
  Robot() {
    x_ = y_ = 0;
    max_dist_ = 1.0;
    max_angle_ = M_PI / 2.0;
    heading_ = 0.0;
  }

  Robot(int x, int y, float dist, float angle, float heading) {
    x_ = x;
    y_ = y;
    max_dist_ = dist;
    max_angle_ = angle;
    heading_ = heading;
  }
  ~Robot() {}

  // *************** state info ***************
  int getPosX() { return x_; }
  int getPosY() { return y_; }
  float getMaxDist() { return max_dist_; }
  float getMaxAngle() { return max_angle_; }
  float getHeading() { return heading_; }

  void setPosX(int x) { x_ = x; }
  void setPosY(int y) { y_ = y; }
  void setHeading(float heading) { heading_ = heading; }

  void setMaxDist(float max_dist) { max_dist_ = max_dist; }
  void setMaxAngle(float max_angle) { max_angle_ = max_angle; }

  // *************** get input frame ***************
  // get all observable laser points
  Map createInputScan(Map map_data);
  // check if a laser point is in the field of view (FoV)
  bool pointInRange(int pt_x, int pt_y);

  // *************** estimate next step ***************
  // 1) if no obstacle in sight (in the direction of target) then go straight in
  // that direction with step size = max_dist_;
  // 2) else if obstacle exists, then check the scan data from heading to +- max
  // angle and see if there're gaps elsewhere. In this case, turn to that
  // direction with step_size_ = 0.

  // return value: 0 - just direction; 1 - next pos estimated
  std::pair<int, int> estimateNextStep(Map scan_data,
                                       std::pair<int, int> end_pos);

  // *************** move ***************
  // update robot states according to next pos
  void move(std::pair<int, int> next_pos);
};

#endif  // ROBOT_H

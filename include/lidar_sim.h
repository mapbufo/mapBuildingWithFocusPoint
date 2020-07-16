#ifndef LIDAR_SIM_H
#define LIDAR_SIM_H

/*
 * coordinate system:
 *
 * o ------- y (heading = 90 deg)
 * |
 * |
 * |
 * x (heading = 0 deg)
 *
 */

#include <math.h>
#include <algorithm>
#include <iostream>
#include <memory>
#include <vector>
#include "map.h"
namespace simulation {
class LaserPoint {
 public:
  typedef std::shared_ptr<LaserPoint> Ptr;

  int x_;
  int y_;

 public:
  LaserPoint() {
    x_ = 0;
    y_ = 0;
  }
  LaserPoint(int x, int y) {
    x_ = x;
    y_ = y;
  }
  void setPositionX(int x) { x_ = x; }
  void setPositionY(int y) { y_ = y; }

  int getPositionX() { return x_; }
  int getPositionY() { return y_; }
};

class LidarSim {
 public:
  typedef std::shared_ptr<LidarSim> Ptr;

  int x_;            // x coordinate of the lidar
  int y_;            // y coordinate of the lidar
  float max_dist_;   // maximum detection range
  float max_angle_;  // maximum detection angle in radius
  float heading_;    // lidar direction:
  // heading = 0 if the center of the lidar is aligned with the x+ axis;
  // heading = PI/2 if aligned with the y+ axis
  // heading = -PI/2 if aligned with the y- axis
  // heading = +-PI if aligned with the x- axis

  LidarSim() {
    x_ = y_ = 0;
    max_dist_ = 1.0;
    max_angle_ = M_PI / 2.0;
    heading_ = 0.0;
  }

  LidarSim(int x, int y, float dist, float angle, float heading) {
    x_ = x;
    y_ = y;
    max_dist_ = dist;
    max_angle_ = angle;
    heading_ = heading;
  }
  ~LidarSim() {}
  // check if a laser point is in the field of view (FoV)

  void setPosX(int x) { x_ = x; }
  void setPosY(int y) { y_ = y; }
  void setHeading(float heading) { heading_ = heading; }
  void setMaxDist(float max_dist) { max_dist_ = max_dist; }
  void setMaxAngle(float max_angle) { max_angle_ = max_angle; }

  int getPosX() { return x_; }
  int getPosY() { return y_; }
  float getHeading() { return heading_; }
  float getMaxDist() { return max_dist_; }
  float getMaxAngle() { return max_angle_; }

  void updatePose(int x, int y, int heading) {
    setPosX(x);
    setPosY(y);
    setHeading(heading);
  }
  // get all observable laser points
  ScanData createInputScan(Map map_data);

  // check if a laser point is in the field of view (FoV)
  bool pointInRange(int pt_x, int pt_y);
};
}  // namespace simulation

#endif  // LIDAR_SIM_H

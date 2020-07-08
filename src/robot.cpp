#include "robot.h"

// check if a laser point is in the field of view (FoV)
bool Robot::pointInRange(int pt_x, int pt_y) {
  // the point must be in the observable range
  // 1. (0, dist)
  // 2. (heading - angle, heading + angle)

  int dist_x = pt_x - x_;
  int dist_y = pt_y - y_;

  // 1. check distance
  float pt_dist = std::sqrt(std::pow(dist_x, 2) + std::pow(dist_y, 2));
  if (pt_dist > max_dist_) return false;

  // 2. check angle
  float pt_angle = std::atan2(dist_y * 1.0, dist_x * 1.0);
  if (std::abs(pt_angle - heading_) > max_angle_) return false;

  return true;
}
// get all observable laser points
std::vector<Point2D> Robot::createInputScan(
    std::vector<std::vector<int>> map_data) {
  std::vector<Point2D> observed_point_list;
  // report error if the map is empty
  if (map_data.size() == 0) {
    std::cerr << "Invalid map data!" << std::endl;
    return observed_point_list;
  }

  // get map size
  int map_size_x = map_data.size();
  int map_size_y = map_data[0].size();

  // get the min & max range in x and y axis
  int min_x = std::max(int(std::floor(x_ - max_dist_)), 0);
  int max_x = std::min(int(std::ceil(x_ + max_dist_)), map_size_x - 1);
  int min_y = std::max(int(std::floor(y_ - max_dist_)), 0);
  int max_y = std::min(int(std::ceil(y_ + max_dist_)), map_size_y - 1);

  // check which points are within the detection range

  for (int i = min_x; i <= max_x; i++) {
    for (int j = min_y; j <= max_y; j++) {
      // check if the laser point lies in the field of view (dist, angle
      // heading)

      if (pointInRange(i, j)) {
        Point2D laser_point(i, j);
        laser_point.setValue(map_data[i][j]);
        observed_point_list.push_back(laser_point);
      }
    }
  }
  return observed_point_list;
}

/*
 *
 * ^ max empty dist
 * |
 * |
 * |
 * | far but wrong
 * |              actual best direction
 * |    |                   |
 * |    |                 | | |
 * |  | | |             | | | | #
 * || | | | | | | | | | | | | | | |
 * || | | | | | | | | | | | | | | |
 * || | | | | | | | | | | | | | | | | | | |
 * || | | | | | | | | | | | | | | | | | | |
 * || | | | | | | | | | | | | | | | | | | | | | | | | | | | |
 *  ----------------------------+-------------------------------> scan angle
 *                            suggested
 *                             heading (middle of the range)
 *
 * #: --> here in this function, this point is selected as the next move.
 */

void Robot::estimateNextStep(std::vector<Point2D> scan_data,
                             float suggested_heading) {
  // if not facing the best direction then turn to that direction
  if (heading_ != suggested_heading) {
    heading_ = suggested_heading;
    return;
  }

  // if already at the best heading:
  // move as far as the robot can in that direction (needs better algo., but
  // enough for now)
  // --> find the farthest, empty scan point in that direction
  Point2D next_pos;                 // initialization
  float next_pos_angle = 2 * M_PI;  // init with some big number
  bool init = false;
  for (auto pt : scan_data) {
    if (pt.getValue() != 0)  // obstacle, can't move there!
      continue;

    if (!init) {  // init next pos with an empty point
      next_pos.setPositionX(pt.getPositionX());
      next_pos.setPositionY(pt.getPositionY());
      int dist_x = next_pos.x_ - x_;
      int dist_y = next_pos.y_ - y_;
      next_pos_angle = std::atan2(dist_y * 1.0, dist_x * 1.0);
    }

    // find the point in the suggested direction
    int dist_x = pt.getPositionX() - x_;
    int dist_y = pt.getPositionY() - y_;
    float pt_angle = std::atan2(dist_y * 1.0, dist_x * 1.0);
    if (pt_angle < next_pos_angle) {
      next_pos = pt;
      next_pos_angle = pt_angle;
    }
  }
  next_pos_ = next_pos;
}

void Robot::move() {
  int dist_x = next_pos_.getPositionX() - x_;
  int dist_y = next_pos_.getPositionY() - y_;
  heading_ = std::atan2(dist_y * 1.0, dist_x * 1.0);
  x_ = next_pos_.getPositionX();
  y_ = next_pos_.getPositionY();
}

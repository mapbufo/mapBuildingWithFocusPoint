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
  if (pt_dist > max_dist_)
    return false;

  // 2. check angle
  float pt_angle = std::atan2(dist_y * 1.0, dist_x * 1.0);
  if (std::abs(pt_angle - heading_) > max_angle_)
    return false;

  return true;
}
// get all observable laser points
Map Robot::createInputScan(Map map_data) {

  Map observed_point_list;
  boost::unordered_map<std::pair<int, int>, CellOccupied> map =
      map_data.GetMap();
  // report error if the map is empty
  if (map.size() == 0) {
    std::cerr << "Invalid map data!" << std::endl;
    return observed_point_list;
  }

  // get map size
  int map_size = map.size();

  // get the min & max range in x and y axis
  int min_x = std::max(int(std::floor(x_ - max_dist_)), 0);
  int max_x = std::min(int(std::ceil(x_ + max_dist_)), map_size - 1);
  int min_y = std::max(int(std::floor(y_ - max_dist_)), 0);
  int max_y = std::min(int(std::ceil(y_ + max_dist_)), map_size - 1);

  // check which points are within the detection range

  for (int i = min_x; i <= max_x; i++) {
    for (int j = min_y; j <= max_y; j++) {
      // check if the laser point lies in the field of view (dist, angle
      // heading)

      if (pointInRange(i, j)) {
        std::pair<int, int> laser_point(i, j);
        observed_point_list.AddPoint(laser_point, map[laser_point]);
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

std::pair<int, int> Robot::estimateNextStep(Map scan_data) {

  boost::unordered_map<std::pair<int, int>, CellOccupied> scan =
      scan_data.GetMap();
  // if already at the best heading:
  // move as far as the robot can in that direction (needs better algo., but
  // enough for now)
  // --> find the farthest, empty scan point in that direction
  std::pair<int, int> best_pos; // initialization
  float best_heading;           // init with some big number
  float heading_diff;
  bool init = false;

  // find points near the line with ratio ~= heading
  float ratio = std::tan(heading_);
  int grid_size = 1;
  int num_steps = int(max_dist_ / grid_size); // value on x axis
  // now compute value on y axis
  for (int i = 1; i <= num_steps; i++) {
    int x;
    if (heading_ > -M_PI / 2.0 && heading_ < M_PI / 2.0)
      x = x_ + i;
    else
      x = x_ - i;
    float y = float(y_) + ratio * float(x);

    // scan point exists:
    // first check floor(y)
    if (scan.find({x, floor(y)}) != scan.end()) {
      // if this point is not occupied
      if (scan[{x, floor(y)}] == CellOccupied::empty) {
        float cur_heading = std::atan2(floor(y), x);
        float cur_heading_diff = std::abs(cur_heading - heading_);
        if (!init) {
          best_heading = cur_heading;
          heading_diff = cur_heading_diff;
          best_pos.first = x;
          best_pos.second = floor(y);

        } else if (cur_heading_diff < heading_diff) { // choose the one with
                                                      // smaller angle
                                                      // difference
          best_heading = cur_heading;
          heading_diff = cur_heading_diff;
          best_pos.first = x;
          best_pos.second = floor(y);
        } else if (cur_heading_diff == heading_diff) // choose the farther point
                                                     // when angle differences
                                                     // are the same
        {
          float best_dist =
              std::sqrt(pow(best_pos.first, 2) + pow(best_pos.second, 2));
          float cur_dist = std::sqrt(pow(x, 2) + pow(floor(y), 2));
          if (cur_dist > best_dist) {
            best_heading = cur_heading;
            heading_diff = cur_heading_diff;
            best_pos.first = x;
            best_pos.second = floor(y);
          }
        }
      }
    }

    // second check ceil(y)
    if (scan.find({x, ceil(y)}) != scan.end()) {
      // if this point is not occupied
      if (scan[{x, ceil(y)}] == CellOccupied::empty) {
        float cur_heading = std::atan2(ceil(y), x);
        float cur_heading_diff = std::abs(cur_heading - heading_);
        if (!init) {
          best_heading = cur_heading;
          heading_diff = cur_heading_diff;
          best_pos.first = x;
          best_pos.second = ceil(y);

        } else if (cur_heading_diff < heading_diff) { // choose the one with
                                                      // smaller angle
                                                      // difference
          best_heading = cur_heading;
          heading_diff = cur_heading_diff;
          best_pos.first = x;
          best_pos.second = ceil(y);
        } else if (cur_heading_diff == heading_diff) // choose the farther point
                                                     // when angle differences
                                                     // are the same
        {
          float best_dist =
              std::sqrt(pow(best_pos.first, 2) + pow(best_pos.second, 2));
          float cur_dist = std::sqrt(pow(x, 2) + pow(ceil(y), 2));
          if (cur_dist > best_dist) {
            best_heading = cur_heading;
            heading_diff = cur_heading_diff;
            best_pos.first = x;
            best_pos.second = ceil(y);
          }
        }
      }
    }
  }

  //  float min_angle_diff;

  //  for (auto pt : scan_data) {
  //    if (pt.getValue() != 0) // obstacle, can't move there!
  //      continue;

  //    if (!init) { // init next pos with an empty point
  //      next_pos.setPositionX(pt.getPositionX());
  //      next_pos.setPositionY(pt.getPositionY());
  //      int dist_x = next_pos.x_ - x_;
  //      int dist_y = next_pos.y_ - y_;
  //      next_pos_angle = std::atan2(dist_y * 1.0, dist_x * 1.0);
  //      min_angle_diff = std::abs(next_pos_angle - heading_);
  //      init = true;
  //    }

  //    // find the point in the suggested direction
  //    int dist_x = pt.getPositionX() - x_;
  //    int dist_y = pt.getPositionY() - y_;
  //    float pt_angle = std::atan2(dist_y * 1.0, dist_x * 1.0);
  //    float pt_angle_diff = std::abs(pt_angle - heading_);
  //    if (pt_angle_diff < min_angle_diff) {
  //      next_pos = pt;
  //      next_pos_angle = pt_angle;
  //      min_angle_diff = pt_angle_diff;

  //    } else if (pt_angle_diff == min_angle_diff) {
  //      float next_pos_dist = std::sqrt(pow(next_pos.getPositionX(), 2) +
  //                                      pow(next_pos.getPositionY(), 2));
  //      float pt_dist =
  //          std::sqrt(pow(pt.getPositionX(), 2) + pow(pt.getPositionY(), 2));

  //      if (pt_dist > next_pos_dist) {
  //        next_pos = pt;
  //        next_pos_angle = pt_angle;
  //      }
  //    }
  //  }

  return best_pos;
}

void Robot::move(std::pair<int, int> next_pos) {
  int dist_x = next_pos.first - x_;
  int dist_y = next_pos.second - y_;
  heading_ = std::atan2(dist_y * 1.0, dist_x * 1.0);
  x_ = next_pos.first;
  y_ = next_pos.second;
}

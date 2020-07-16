#include "robot.h"

// find the best position based on the input scan
/// 1. if only interested in empty points, then only empty points are considered
/// 2. if interested in the overall best point, all points are then considered
void Robot::findBestPos(ScanData scan, Point2D candidate_pt, Point2D &best_pos,
                        float &best_heading, bool only_empty) {
  bool init = false;
  float best_heading_diff = 10e5;
  int pt_x = candidate_pt.first;
  int pt_y = candidate_pt.second;
  std::set<CellOccupied> interested_status_list;
  if (only_empty) {
    interested_status_list.insert(CellOccupied::empty);
  } else {
    interested_status_list.insert(CellOccupied::empty);
    interested_status_list.insert(CellOccupied::path);
    interested_status_list.insert(CellOccupied::unknown);
    interested_status_list.insert(CellOccupied::occupied);
    interested_status_list.insert(CellOccupied::target_pos);
    interested_status_list.insert(CellOccupied::robot_pos);
  }

  if (scan.find({pt_x, pt_y}) != scan.end()) {
    // if this point is not occupied
    if (interested_status_list.find(scan[candidate_pt]) !=
        interested_status_list.end()) {
      float cur_heading = std::atan2(pt_y, pt_x);
      float cur_heading_diff = std::abs(cur_heading - heading_);
      if (!init) {
        best_heading = cur_heading;
        best_heading_diff = cur_heading_diff;
        best_pos.first = pt_x;
        best_pos.second = pt_y;
        init = true;
      } else if (cur_heading_diff < best_heading_diff) {  // choose the one with
                                                          // smaller angle
                                                          // difference
        best_heading = cur_heading;
        best_heading_diff = cur_heading_diff;
        best_pos.first = pt_x;
        best_pos.second = pt_y;
      } else if (cur_heading_diff ==
                 best_heading_diff)  // choose the farther point
                                     // when angle differences
                                     // are the same
      {
        float best_dist =
            std::sqrt(pow(best_pos.first, 2) + pow(best_pos.second, 2));
        float cur_dist = std::sqrt(pow(pt_x, 2) + pow(pt_y, 2));
        if (cur_dist > best_dist) {
          best_heading = cur_heading;
          best_heading_diff = cur_heading_diff;
          best_pos.first = pt_x;
          best_pos.second = pt_y;
        }
      }
    }
  }
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

Point2D Robot::estimateNextStep(ScanData scan, Point2D end_pos, float max_dist,
                                bool &call_help) {
  //  for (auto sc : scan) {
  //    std::cout << sc.first.first << " " << sc.first.second << std::endl;
  //  }

  // first check if the end pos is in reach
  if (scan.find(end_pos) != scan.end()) {
    if (scan[end_pos] == CellOccupied::empty) return end_pos;
  }

  // if already at the best heading:
  // move as far as the robot can in that direction (needs better algo., but
  // enough for now)
  // --> find the farthest, empty scan point in that direction
  Point2D farthest_pos;
  float farthest_pos_heading;
  Point2D best_pos(-100, -100);  // initialization
  float best_heading;            // init with some big number

  // find points near the line with ratio ~= heading
  float ratio = 0.0;
  if (std::abs(std::abs(heading_) - M_PI / 2.0) < 10e-2) {
    ratio = 0;
  } else {
    ratio = tan(heading_);
  }

  int grid_size = 1;
  int num_steps = int(max_dist / grid_size);  // value on x axis

  std::set<std::pair<int, int>> line_point_vec;
  // go in x / y direction

  // 1. x direction: for each x on grid, there's a y value on the line. Get this
  // y value!
  {
    for (int i = 1; i <= num_steps; i++) {
      float y;
      float x;
      if (std::abs(heading_) - M_PI / 2.0 < -10e-2) {
        x = float(x_ + i);
        y = (x - x_) * ratio + float(y_);

      } else if (std::abs(heading_) - M_PI / 2.0 > 10e-2) {
        x = float(x_ - i);
        y = (x - x_) * ratio + float(y_);
      } else  // heading == +- M_PI
      {
        x = x_;
        if (heading_ > 0)
          y = float(y_ + i);
        else
          y = float(y_ - i);
      }
      line_point_vec.insert({x, floor(y)});
      line_point_vec.insert({x, ceil(y)});
    }

    // 2. y direction: for each y on grid, there's an x value on the line. Get
    // this x value!

    {
      for (int i = 1; i <= num_steps; i++) {
        float y;
        float x;
        if (heading_ > 10e-2 &&
            heading_ - M_PI / 2.0 < -10e-2) {  // 1st quadrant
          y = float(y_ + i);
          x = (y - y_) / ratio + float(x_);
          line_point_vec.insert({floor(x), y});
          line_point_vec.insert({ceil(x), y});
        } else if (heading_ < -10e-2 &&
                   heading_ + M_PI / 2.0 < -10e-2) {  // 4th quadrant
          y = float(y_ - i);
          x = (y - y_) / ratio + float(x_);
          line_point_vec.insert({floor(x), y});
          line_point_vec.insert({ceil(x), y});
        } else if (M_PI - heading_ > 10e-2 &&
                   heading_ - M_PI / 2.0 > 10e-2) {  // 2nd quadrant
          y = float(y_ + i);
          x = (y - y_) / ratio + float(x_);

          line_point_vec.insert({floor(x), y});
          line_point_vec.insert({ceil(x), y});
        } else if (M_PI + heading_ > 10e-2 && heading_ + M_PI / 2.0 < -10e-2) {
          y = float(y_ - i);
          x = (y - y_) / ratio + float(x_);

          line_point_vec.insert({floor(x), y});
          line_point_vec.insert({ceil(x), y});

        } else if (std::abs(std::abs(heading_) - M_PI / 2.0) <
                   10e-2)  // heading == +- M_PI
        {
          x = x_;
          if (heading_ > 0)
            y = float(y_ + i);
          else
            y = float(y_ - i);
          line_point_vec.insert({x, floor(y)});
          line_point_vec.insert({x, ceil(y)});
        } else  // heading == 0 / M_PI
        {
          y = y_;
          if (std::abs(heading_) < 10e-2) {
            x = float(x_ + i);

          } else {
            x = float(x_ - i);
          }
          line_point_vec.insert({floor(x), y});
          line_point_vec.insert({ceil(x), y});
        }
      }
    }

    // scan point exists:
    for (auto pt : line_point_vec) {
      int pt_x = pt.first;
      int pt_y = pt.second;
      if (pt_x < 0 || pt_y < 0) continue;
      findBestPos(scan, pt, best_pos, best_heading, true);
      findBestPos(scan, pt, farthest_pos, farthest_pos_heading, false);
    }
  }

  if (best_pos.first == -100 && best_pos.second == -100) {
    best_pos.first = x_;
    best_pos.second = y_;
    best_heading = heading_;
  }
  if (best_pos.first != farthest_pos.first ||
      best_pos.second != farthest_pos.second) {
    call_help = true;
  }

  return best_pos;
}

void Robot::move(std::pair<int, int> next_pos) {
  int dist_x = next_pos.first - x_;
  int dist_y = next_pos.second - y_;
  heading_ = std::atan2(dist_y * 1.0, dist_x * 1.0);
  x_ = next_pos.first;
  y_ = next_pos.second;
}

#include "robot.h"

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

std::pair<int, int> Robot::estimateNextStep(Map scan_data,
                                            std::pair<int, int> end_pos,
                                            float max_dist) {
  boost::unordered_map<std::pair<int, int>, CellOccupied> scan =
      scan_data.GetMap();

  //  for (auto sc : scan) {
  //    std::cout << sc.first.first << " " << sc.first.second << std::endl;
  //  }

  // first check if the end pos is in reach
  if (scan.find(end_pos) != scan.end()) {
    return end_pos;
  }

  // if already at the best heading:
  // move as far as the robot can in that direction (needs better algo., but
  // enough for now)
  // --> find the farthest, empty scan point in that direction
  std::pair<int, int> best_pos; // initialization
  float best_heading;           // init with some big number
  float heading_diff;
  bool init = false;

  // find points near the line with ratio ~= heading
  float ratio = 0.0;
  if (std::abs(std::abs(heading_) - M_PI / 2.0) < 10e-2) {
    ratio = 0;
  } else {
    ratio = tan(heading_);
  }

  int grid_size = 1;
  int num_steps = int(max_dist / grid_size); // value on x axis

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
      } else // heading == +- M_PI
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
            heading_ - M_PI / 2.0 < -10e-2) { // 1st quadrant
          y = float(y_ + i);
          x = (y - y_) / ratio + float(x_);
          line_point_vec.insert({floor(x), y});
          line_point_vec.insert({ceil(x), y});
        } else if (heading_ < -10e-2 &&
                   heading_ + M_PI / 2.0 < -10e-2) { // 4th quadrant
          y = float(y_ - i);
          x = (y - y_) / ratio + float(x_);
          line_point_vec.insert({floor(x), y});
          line_point_vec.insert({ceil(x), y});
        } else if (M_PI - heading_ > 10e-2 &&
                   heading_ - M_PI / 2.0 > 10e-2) { // 2nd quadrant
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
                   10e-2) // heading == +- M_PI
        {
          x = x_;
          if (heading_ > 0)
            y = float(y_ + i);
          else
            y = float(y_ - i);
          line_point_vec.insert({x, floor(y)});
          line_point_vec.insert({x, ceil(y)});
        } else // heading == 0 / M_PI
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
      if (pt_x < 0 || pt_y < 0)
        continue;

      if (scan.find({pt_x, pt_y}) != scan.end()) {
        // if this point is not occupied
        if (scan[{pt_x, pt_y}] == CellOccupied::empty) {
          float cur_heading = std::atan2(pt_y, pt_x);
          float cur_heading_diff = std::abs(cur_heading - heading_);
          if (!init) {
            best_heading = cur_heading;
            heading_diff = cur_heading_diff;
            best_pos.first = pt_x;
            best_pos.second = pt_y;

          } else if (cur_heading_diff < heading_diff) { // choose the one with
                                                        // smaller angle
                                                        // difference
            best_heading = cur_heading;
            heading_diff = cur_heading_diff;
            best_pos.first = pt_x;
            best_pos.second = pt_y;
          } else if (cur_heading_diff ==
                     heading_diff) // choose the farther point
                                   // when angle differences
                                   // are the same
          {
            float best_dist =
                std::sqrt(pow(best_pos.first, 2) + pow(best_pos.second, 2));
            float cur_dist = std::sqrt(pow(pt_x, 2) + pow(pt_y, 2));
            if (cur_dist > best_dist) {
              best_heading = cur_heading;
              heading_diff = cur_heading_diff;
              best_pos.first = pt_x;
              best_pos.second = pt_y;
            }
          }
        }
      }
    }
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

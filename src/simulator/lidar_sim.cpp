#include "lidar_sim.h"

namespace simulation {
// check if a laser point is in the field of view (FoV)
// check if a laser point is in the field of view (FoV)
bool LidarSim::pointInRange(int pt_x, int pt_y) {
  // the point must be in the observable range
  // 1. (0, dist)
  // 2. (heading - angle, heading + angle)
  int dist_x = pt_x - x_;
  int dist_y = pt_y - y_;

  // 0. exclude robot pos
  if (dist_x == 0 && dist_y == 0)
    return false;
  // 1. check distance
  float pt_dist = std::sqrt(std::pow(dist_x, 2) + std::pow(dist_y, 2));
  if (pt_dist > max_dist_)
    return false;

  // 2. check angle
  float pt_angle = std::atan2(dist_y * 1.0, dist_x * 1.0);
  float angle_diff = pt_angle - heading_;
  while (std::abs(angle_diff) >= M_PI) {
    if (angle_diff > 0) {
      angle_diff -= 2 * M_PI;
    } else {
      angle_diff += 2 * M_PI;
    }
  }
  if (std::abs(angle_diff) - max_angle_ > 10e-2) {
    return false;
  }
  return true;
}
// get all observable laser points
Map LidarSim::createInputScan(Map map_data) {

  typedef std::pair<int, int> POINT2D;
  Map local_map;
  boost::unordered_map<POINT2D, CellOccupied> map = map_data.GetMap();
  // report error if the map is empty
  if (map.size() == 0) {
    std::cerr << "Invalid map data!" << std::endl;
    return local_map;
  }

  // get map size
  int map_size = map.size();

  // get the min & max range in x and y axis
  int min_x = std::max(int(std::floor(x_ - max_dist_)), 0);
  int max_x = std::min(int(std::ceil(x_ + max_dist_)), map_size - 1);
  int min_y = std::max(int(std::floor(y_ - max_dist_)), 0);
  int max_y = std::min(int(std::ceil(y_ + max_dist_)), map_size - 1);

  // 1. check which points are within the detection range

  std::set<POINT2D> obstacle_list;
  for (int i = min_x; i <= max_x; i++) {
    for (int j = min_y; j <= max_y; j++) {
      // check if the laser point lies in the field of view (dist, angle
      // heading)

      if (pointInRange(i, j)) {
        POINT2D laser_point(i, j);
        local_map.AddPoint(laser_point, map[laser_point]);

        if (map[laser_point] == CellOccupied::occupied)
          obstacle_list.insert(laser_point);
      }
    }
  }
  boost::unordered_map<POINT2D, CellOccupied> local_map_info =
      local_map.GetMap();

  // 2. points on the line of obstacles could be blocked
  for (auto pt : local_map_info) {

    int pt_type = pt.second;
    if (pt_type == CellOccupied::empty) {
      float pt_angle = atan2(pt.first.second - y_, pt.first.first - x_);
      for (auto obstacle : obstacle_list) {
        float obstacle_angle = atan2(obstacle.second - y_, obstacle.first - x_);
        if (std::abs(pt_angle - obstacle_angle) < 10e-5) {

          // if so, check distance
          float pt_dist =
              sqrt(pow(pt.first.first - x_, 2) + pow(pt.first.second - y_, 2));
          float obstacle_dist =
              sqrt(pow(obstacle.first - x_, 2) + pow(obstacle.second - y_, 2));
          if (pt_dist - obstacle_dist > 10e-5) {

            local_map.Update(pt.first.first, pt.first.second,
                             CellOccupied::unknown);
            //            std::cout << "pt: " << pt.first.first << " " <<
            //            pt.first.second
            //                      << " " << pt_angle - obstacle_angle <<
            //                      std::endl;
            //            std::cout << "obs: " << obstacle.first << " " <<
            //            obstacle.second
            //                      << std::endl;
          }
        }
      }
    }
  }

  // 3. get all obstacle point pairs
  std::vector<std::pair<POINT2D, POINT2D>> obstacle_pair_list;
  for (auto obstacle_iter = obstacle_list.begin();
       obstacle_iter != obstacle_list.end(); obstacle_iter++) {
    // check up, down, left, right if there's an obstacle there
    int pos_x = obstacle_iter->first;
    int pos_y = obstacle_iter->second;

    POINT2D point_up(pos_x - 1, pos_y);
    POINT2D point_down(pos_x + 1, pos_y);
    POINT2D point_left(pos_x, pos_y - 1);
    POINT2D point_right(pos_x, pos_y + 1);

    if (obstacle_list.find(point_up) != obstacle_list.end()) {
      obstacle_pair_list.push_back(
          std::pair<POINT2D, POINT2D>(*obstacle_iter, point_up));
    }

    if (obstacle_list.find(point_down) != obstacle_list.end()) {
      obstacle_pair_list.push_back(
          std::pair<POINT2D, POINT2D>(*obstacle_iter, point_down));
    }

    if (obstacle_list.find(point_left) != obstacle_list.end()) {
      obstacle_pair_list.push_back(
          std::pair<POINT2D, POINT2D>(*obstacle_iter, point_left));
    }

    if (obstacle_list.find(point_right) != obstacle_list.end()) {
      obstacle_pair_list.push_back(
          std::pair<POINT2D, POINT2D>(*obstacle_iter, point_right));
    }
    obstacle_list.erase(obstacle_iter);
  }

  // 4. filter out points that are blocked
  for (auto pt : local_map_info) {

    // check all obstacle pairs by min and max blocked angle
    // if a point's angle lies in the range, check distance -
    //    if shorter than the obstacle, then set to empty
    //    else, blocked, set to unknown
    // else check next obstacle pair
    int pt_type = pt.second;
    float pt_angle = atan2(pt.first.second - y_, pt.first.first - x_);
    if (pt_type != CellOccupied::unknown &&
        pt_type != CellOccupied::robot_pos) {
      for (auto obstacle_pair : obstacle_pair_list) {
        if (local_map_info[obstacle_pair.first] == CellOccupied::unknown ||
            local_map_info[obstacle_pair.second] == CellOccupied::unknown)
        // already overwritten
        {
          continue;
        }
        POINT2D obstacle_1 = obstacle_pair.first;
        POINT2D obstacle_2 = obstacle_pair.second;
        if (pt.first == obstacle_1 || pt.first == obstacle_2)
          continue;
        float obstacle_angle_1 =
            atan2(obstacle_1.second - y_, obstacle_1.first - x_);
        float obstacle_angle_2 =
            atan2(obstacle_2.second - y_, obstacle_2.first - x_);
        float max_obstacle_angle = std::max(obstacle_angle_1, obstacle_angle_2);
        float min_obstacle_angle = std::min(obstacle_angle_1, obstacle_angle_2);
        if (pt_angle <= max_obstacle_angle && pt_angle >= min_obstacle_angle) {
          float pt_dist =
              sqrt(pow(pt.first.first - x_, 2) + pow(pt.first.second - y_, 2));
          float obstacle_1_dist = sqrt(pow(obstacle_1.first - x_, 2) +
                                       pow(obstacle_1.second - y_, 2));
          float obstacle_2_dist = sqrt(pow(obstacle_2.first - x_, 2) +
                                       pow(obstacle_2.second - y_, 2));
          if (pt_dist >= std::min(obstacle_1_dist, obstacle_2_dist)) {
            local_map.Update(pt.first.first, pt.first.second,
                             CellOccupied::unknown);
            break;
          }
        }
      }
    }
  }

  return local_map;
}

} // namespace simulation

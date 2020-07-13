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

} // namespace simulation

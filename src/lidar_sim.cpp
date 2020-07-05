#include "lidar_sim.h"

namespace simulation {
// check if a laser point is in the field of view (FoV)
bool LidarSim::pointInRange(int pt_x, int pt_y) {
  // the point must be in the observable range
  // 1. (0, dist)
  // 2. (heading - angle, heading + angle)

  int dist_x = pt_x - x_;
  int dist_y = pt_y - y_;
  float min_view_angle = heading_ - max_angle_;
  while (min_view_angle > 2 * PI || min_view_angle < -2 * PI) {
    if (min_view_angle > 2 * PI)
      min_view_angle -= 2 * PI;
    if (min_view_angle < -2 * PI)
      min_view_angle += 2 * PI;
  }
  float max_view_angle = heading_ + max_angle_;
  while (max_view_angle > 2 * PI || max_view_angle < -2 * PI) {
    if (max_view_angle > 2 * PI)
      max_view_angle -= 2 * PI;
    if (max_view_angle < -2 * PI)
      max_view_angle += 2 * PI;
  }

  // 1. check distance
  float pt_dist = std::sqrt(std::pow(dist_x, 2) + std::pow(dist_y, 2));
  if (pt_dist > max_dist_)
    return false;

  // 2. check angle
  float pt_angle = std::atan2(dist_y * 1.0, dist_x * 1.0);
  if (pt_angle < min_view_angle || pt_angle > max_view_angle)
    return false;

  return true;
}
// get all observable laser points
std::vector<LaserPoint>
LidarSim::collectObservedLaserPoints(std::vector<std::vector<int>> map_data) {
  std::vector<LaserPoint> observed_point_list;
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

      if (pointInRange(i, j) && map_data[i][j] == 1) {
        LaserPoint laser_point(i, j);
        observed_point_list.push_back(laser_point);
      }
    }
  }
  return observed_point_list;
}
}

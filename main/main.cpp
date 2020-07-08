#include <iostream>

#include "lidar_sim.h"
#include "map_simulator.h"

bool ptObserved(std::vector<simulation::LaserPoint> pt_list, int i, int j) {
  for (int k = 0; k < pt_list.size(); k++) {
    simulation::LaserPoint pt = pt_list[k];
    if (i == pt.getPositionX() && j == pt.getPositionY()) {
      return true;
    }
  }
  return false;
}

int main() {
  std::vector<std::vector<int>> map;
  if (status::Error == simulation::LoadMap("../../maps/scenario_03.txt", map)) {
    std::cerr << "Invalid map data!" << std::endl;
  }

  simulation::LidarSim lidar_sim(5, 3, 2.0, M_PI / 4.0, M_PI / 4.0);

  std::vector<simulation::LaserPoint> observed_point_list;
  observed_point_list = lidar_sim.collectObservedLaserPoints(map);

  int map_length = map.size();
  int map_width = map[0].size();

  for (int i = 0; i < map_length; i++) {
    for (int j = 0; j < map_width; j++) {
      if (ptObserved(observed_point_list, i, j)) {
        map[i][j] = 2;
      }
      if (i == lidar_sim.x_ && j == lidar_sim.y_) {
        map[i][j] = 5;
      }
    }
  }

  for (int i = 0; i < map_length; i++) {
    for (int j = 0; j < map_width; j++) {
      std::cout << map[i][j] << " ";
    }
    std::cout << std::endl;
  }

  return 0;
}

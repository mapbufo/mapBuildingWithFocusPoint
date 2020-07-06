#include "lidar_sim.h"
#include "map_simulator.h"
#include <iostream>

int main() {
  std::vector<std::vector<int>> map;
  if (status::Error == simulation::LoadMap("../../maps/scenario_01.txt", map)) {
    std::cerr << "Invalid map data!" << std::endl;
  }

  return 0;
}

#include "map_simulator.h"

namespace simulation {

mapSim::mapSim(std::string path_to_map) {
  // open the map file
  std::ifstream infile;
  infile.open(path_to_map.c_str());

  // fill the map data into the 2d array
  for (int i = 0; i < 10; i++) {
    for (int j = 0; j < 10; j++) {
      infile >> map_array_[i][j];
    }
  }
  infile.close();
}

status mapSim::CreateANewMap(std::string save_path_to_map) { return Undifined; }

status mapSim::LoadMap(std::string path_to_map) {
  // open the map file
  std::ifstream infile;
  infile.open(path_to_map.c_str());

  // fill the map data into the 2d array
  for (int i = 0; i < 10; i++) {
    for (int j = 0; j < 10; j++) {
      infile >> map_array_[i][j];
    }
  }
  infile.close();
  return Ok;
}
} // namespace simulation

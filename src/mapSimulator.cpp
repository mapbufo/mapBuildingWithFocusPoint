#include "mapSimulator.h"

namespace simulation {

map_sim::map_sim(std::string path_to_map) {
  // open the map file
  std::ifstream infile;
  infile.open(path_to_map);

  // fill the map data into the 2d array
  for (int i = 0; i < 1000; i++) {
    for (int j = 0; j < 1000; j++) {
      infile >> map_array[i][j];
    }
  }
  infile.close();
}

status map_sim::creatANewMap(std::string save_path_to_map) { return Undifined; }

status map_sim::loadMap(std::string path_to_map) {
  // open the map file
  std::ifstream infile;
  infile.open(path_to_map);

  // fill the map data into the 2d array
  for (int i = 0; i < 1000; i++) {
    for (int j = 0; j < 1000; j++) {
      infile >> map_array[i][j];
    }
  }
  infile.close();
  return Ok;
}
} // namespace simulation

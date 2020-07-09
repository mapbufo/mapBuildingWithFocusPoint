#include "map_simulator.h"

namespace simulation {

status::status LoadMap(std::string path_to_map,
                       std::vector<std::vector<int>> &map) {
  // open the map file
  std::ifstream infile;
  infile.open(path_to_map.c_str());

  // fill the map data into the 2d vector
  std::vector<int> map_row;
  int tmp;
  for (int i = 0; i < 10; i++) {
    for (int j = 0; j < 10; j++) {
      infile >> tmp;
      map_row.push_back(tmp);
    }
    map.push_back(map_row);
    map_row.clear();
  }
  infile.close();
  return status::Ok;
}
}  // namespace simulation

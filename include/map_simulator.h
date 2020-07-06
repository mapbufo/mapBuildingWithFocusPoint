#include <fstream>
#include <iostream>
#include <string>

namespace simulation {

enum status { Error = 0, Ok = 1, Undifined = 2 };

class mapSim {
public:
  mapSim(std::string path_to_map);
  ~mapSim(){};
  status CreateANewMap(std::string save_path_to_map);

  status LoadMap(std::string path_to_map);

protected:
  int map_array_[10][10];
};

} // namespace simulation

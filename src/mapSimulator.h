#include <fstream>
#include <iostream>
#include <string>

namespace simulation {

enum status { Error = 0, Ok = 1, Undifined = 2 };

class map_sim {
public:
  map_sim(std::string path_to_map);
  ~map_sim();
  status creatANewMap(std::string save_path_to_map);

  status loadMap(std::string path_to_map);

protected:
  int map_array[1000][1000];
};

} // namespace simulation
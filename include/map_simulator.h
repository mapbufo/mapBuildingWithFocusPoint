#ifndef MAP_SIMULATOR_H
#define MAP_SIMULATOR_H
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace status {
enum status { Error = 0, Ok = 1, Undifined = 2 };
}
namespace simulation {
status::status LoadMap(std::string path_to_map,
                       std::vector<std::vector<int>> &map);

} // namespace simulation
#endif // !MAP_SIMULATOR_H
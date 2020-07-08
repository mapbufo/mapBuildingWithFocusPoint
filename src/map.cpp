#include "map.h"

boost::unordered_map<std::pair<int, int>, CellOccupied> Map::GetMap() {
  return map_;
}

CellOccupied Map::GetCell(int x, int y) {
  if (map_.count({x, y}) == 0) {
    return CellOccupied::unknown;
  }
  return map_[{x, y}];
}

status::status Map::Update(int x, int y, CellOccupied occupied) {
  if (x >= size_of_map_ || y >= size_of_map_) {
    std::cerr << "The position is over the size of map!" << std::endl;
    return status::Error;
  }
  map_[{x, y}] = occupied;
  return status::Ok;
}

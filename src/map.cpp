#include "map.h"

boost::unordered_map<std::pair<int, int>, CellOccupied> Map::GetMap() {
  return map_;
}

CellOccupied Map::GetCell(int x, int y) {
  if (x >= size_of_map_ || x < 0 || y >= size_of_map_ || y < 0) {
    return CellOccupied::out_of_map;
  }
  if (map_.count({x, y}) == 0) {
    return CellOccupied::unknown;
  }
  return map_[{x, y}];
}

CellOccupied Map::GetCell(Point2D pos) {
  return GetCell(pos.first, pos.second);
}

status::status Map::Update(int x, int y, CellOccupied occupied) {
  if (x >= size_of_map_ || y >= size_of_map_) {
    std::cerr << "The position is over the size of map!" << std::endl;
    return status::Error;
  }
  map_[{x, y}] = occupied;
  return status::Ok;
}

status::status Map::Update(Point2D pos, CellOccupied occupied) {
  return Update(pos.first, pos.second, occupied);
}

void Map::PrintMap() {
  for (int i = 0; i < size_of_map_; i++) {
    for (int j = 0; j < size_of_map_; j++) {
      std::cerr << map_[{i, j}] << " ";
    }
    std::cerr << std::endl;
  }
}

status::status Map::Load(std::string path_to_map) {
  // open the map file
  std::ifstream infile;
  infile.open(path_to_map.c_str());

  // fill the map data into the map_
  int tmp;
  for (int i = 0; i < size_of_map_; i++) {
    for (int j = 0; j < size_of_map_; j++) {
      infile >> tmp;
      switch (tmp) {
        case 0:
          map_[{i, j}] = CellOccupied::empty;
          break;
        case 1:
          map_[{i, j}] = CellOccupied::occupied;
          break;
        case 5:
          map_[{i, j}] = CellOccupied::robot_pos;
          break;
        default:
          map_[{i, j}] = CellOccupied::unknown;
          break;
      }
    }
  }
  infile.close();
  return status::Ok;
}
#include "map.h"

boost::unordered_map<std::pair<int, int>, CellOccupied> &Map::GetMap() {
  return map_;
}

CellOccupied Map::GetCell(int x, int y) {
  if (map_.count({x, y}) == 0) {
    return CellOccupied::unknown;
  }
  return map_[{x, y}];
}

CellOccupied Map::GetCell(Point2D pos) {
  return GetCell(pos.first, pos.second);
}

status::status Map::Update(int x, int y, CellOccupied occupied) {
  map_[{x, y}] = occupied;

  // update map size if new point pos is not in the old map
  if (x < map_x_min_) {
    map_x_min_ = x;
  }
  if (x > map_x_max_) {
    map_x_max_ = x;
  }
  if (y < map_y_min_) {
    map_y_min_ = y;
  }
  if (y > map_y_max_) {
    map_y_max_ = y;
  }
  size_of_map_.first = map_x_max_ - map_x_min_ + 1;
  size_of_map_.second = map_y_max_ - map_y_min_ + 1;
  return status::Ok;
}

status::status Map::Update(Point2D pos, CellOccupied occupied) {
  return Update(pos.first, pos.second, occupied);
}

void Map::PrintMap() {
  // Actually this should iterates through map_, not by size_of_map_, but right
  // now this is used because this is needed for printing in Terminal.
  for (int i = map_x_min_; i < map_x_max_; i++) {
    for (int j = map_y_min_; j < map_y_max_; j++) {
      std::cerr << GetCell(i, j) << " ";
    }
    std::cerr << std::endl;
  }
}

// TODO(YiLuo) : should not load by the size of map, but by the values in map
status::status Map::Load(std::string path_to_map) {
  // open the map file
  std::ifstream infile;
  infile.open(path_to_map.c_str());

  // fill the map data into the map_
  int tmp;
  for (int i = 0; i < size_of_map_.first; i++) {
    for (int j = 0; j < size_of_map_.second; j++) {
      infile >> tmp;
      switch (tmp) {
        case 0:
          map_[{i, j}] = CellOccupied::empty;
          break;
        case 1:
          map_[{i, j}] = CellOccupied::occupied;
          break;
        case 2:
          map_[{i, j}] = CellOccupied::unknown;
          break;
        case 4:
          map_[{i, j}] = CellOccupied::path;
          break;
        case 5:
          map_[{i, j}] = CellOccupied::robot_pos;
          break;
        case 6:
          map_[{i, j}] = CellOccupied::target_pos;
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


// TODO(YiLuo) : should not load by the size of map, but by the values in map
status::status Map::LoadGlobalMap(std::string path_to_map) {
  // open the map file
  std::ifstream infile;
  infile.open(path_to_map.c_str());

  // fill the map data into the map_
  int tmp;
  for (int i = - size_of_map_.first / 2; i < size_of_map_.first / 2; i++) {
    for (int j = - size_of_map_.second / 2; j < size_of_map_.second / 2; j++) {
      infile >> tmp;
      switch (tmp) {
        case 0:
          map_[{i, j}] = CellOccupied::empty;
          break;
        case 1:
          map_[{i, j}] = CellOccupied::occupied;
          break;
        case 2:
          map_[{i, j}] = CellOccupied::unknown;
          break;
        case 4:
          map_[{i, j}] = CellOccupied::path;
          break;
        case 5:
          map_[{i, j}] = CellOccupied::robot_pos;
          break;
        case 6:
          map_[{i, j}] = CellOccupied::target_pos;
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
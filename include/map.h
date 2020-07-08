#ifndef MAP_H
#define MAP_H

#include <boost/unordered_map.hpp>
#include <iostream>

#include "common.h"

enum CellOccupied { empty = 0, occupied = 1, unknown = 2 };

class Map {
 public:
  Map() : size_of_map_(10){};
  ~Map();

  boost::unordered_map<std::pair<int, int>, CellOccupied> GetMap();
  CellOccupied GetCell(int x, int y);
  status::status Update(int x, int y, CellOccupied occupied);

 private:
  boost::unordered_map<std::pair<int, int>, CellOccupied> map_;
  int size_of_map_;
};

#endif  // !MAP_H
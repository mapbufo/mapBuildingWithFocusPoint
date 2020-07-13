#ifndef MAP_H
#define MAP_H

#include <boost/unordered_map.hpp>
#include <fstream>
#include <iostream>

#include "common.h"

enum CellOccupied { empty = 0, occupied = 1, unknown = 2, robot_pos = 5 };

class Map {
public:
  // used to create global map
  Map(int size_of_map) { size_of_map_ = size_of_map; };
  // used to create local scan point map (not likely to be a square so size of
  // map does not make any sense :))
  Map() { size_of_map_ = -1; };

  ~Map(){};

  boost::unordered_map<std::pair<int, int>, CellOccupied> &GetMap();
  CellOccupied GetCell(int x, int y);
  status::status Update(int x, int y, CellOccupied occupied);

  void PrintMap();
  status::status AddPoint(std::pair<int, int> pos, CellOccupied cellOccupied) {
    map_.insert({pos, cellOccupied});
    return status::Ok;
  };
  // only for simulation
  status::status Load(std::string path_to_map);

private:
  boost::unordered_map<std::pair<int, int>, CellOccupied> map_;

  int size_of_map_;
};

#endif // MAP_H

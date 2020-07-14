#ifndef MAP_H
#define MAP_H

#include <boost/unordered_map.hpp>
#include <fstream>
#include <iostream>

#include "common.h"

enum CellOccupied {
  empty = 0,
  occupied = 1,
  unknown = 2,
  out_of_map = 3,
  path = 4,
  robot_pos = 5,
  target_pos = 6
};

class Map {
 public:
  Map() : size_of_map_(10){};
  ~Map(){};

  boost::unordered_map<std::pair<int, int>, CellOccupied> GetMap();
  CellOccupied GetCell(int x, int y);
  CellOccupied GetCell(Point2D pos);
  status::status Update(int x, int y, CellOccupied occupied);
  status::status Update(Point2D pos, CellOccupied occupied);
  void PrintMap();

  // only for simulation
  status::status Load(std::string path_to_map);

 private:
  boost::unordered_map<std::pair<int, int>, CellOccupied> map_;
  int size_of_map_;
};

#endif  // !MAP_H
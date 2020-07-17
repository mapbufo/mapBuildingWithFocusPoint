#ifndef MAP_H
#define MAP_H

#include <boost/unordered_map.hpp>
#include <fstream>
#include <iostream>

#include "common.h"

class Map {
public:
  // used to create global map
  Map(std::pair<int, int> size_of_map) {
    size_of_map_ = size_of_map;
    map_x_min_ = 0;
    map_x_max_ = size_of_map_.first;
    map_y_min_ = 0;
    map_y_max_ = size_of_map_.second;
  };

  ~Map(){};

  CellOccupied operator[](Point2D point) { return GetCell(point); };

  boost::unordered_map<std::pair<int, int>, CellOccupied> &GetMap();
  CellOccupied GetCell(int x, int y);
  CellOccupied GetCell(Point2D pos);
  status::status Update(int x, int y, CellOccupied occupied);
  status::status Update(Point2D pos, CellOccupied occupied);
  int GetMapSizeXMin() { return map_x_min_; }
  int GetMapSizeXMax() { return map_x_max_; }
  int GetMapSizeYMin() { return map_y_min_; }
  int GetMapSizeYMax() { return map_y_max_; }

  std::pair<int, int> GetMapSize() { return size_of_map_; }
  void PrintMap();
  // only for simulation
  status::status Load(std::string path_to_map);
  status::status LoadGlobalMap(std::string path_to_map);

private:
  boost::unordered_map<std::pair<int, int>, CellOccupied> map_;

  std::pair<int, int> size_of_map_;
  int map_x_min_;
  int map_x_max_;
  int map_y_min_;
  int map_y_max_;

  void computeMapSize();
};

#endif // MAP_H

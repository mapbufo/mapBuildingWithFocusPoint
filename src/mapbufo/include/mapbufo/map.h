#ifndef MAP_H
#define MAP_H

#include <boost/unordered_map.hpp>
#include <fstream>
#include <iostream>

#include "common.h"
#include <nav_msgs/OccupancyGrid.h>

class Map {
public:
  // used to create global map
  Map() {
    // set the frame as default "world"
    map_.header.frame_id = "world";
    // 10m x 10m, with resolution 10cm
    map_.info.resolution = 0.1;
    map_.info.height = 100;
    map_.info.width = 100;
    map_.info.origin.position.x = 0.0;
    map_.info.origin.position.y = 0.0;
    map_.info.origin.position.z = 0.0;
    map_.info.origin.orientation.x = 0.0;
    map_.info.origin.orientation.y = 0.0;
    map_.info.origin.orientation.z = 0.0;
    map_.info.origin.orientation.w = 1.0;

    // initialize map_data to -1
    // use row-major to save the data, index = y*width + x
    map_.data.insert(begin(map_.data), map_.info.width * map_.info.height, -1);
  }
  Map(uint32_t width, uint32_t height) {
    map_.info.width = width;
    map_.info.height = height;
  };

  ~Map(){};

  CellOccupied operator[](Point2D point) { return GetCell(point); };

  nav_msgs::OccupancyGrid &GetMap();
  CellOccupied GetCell(int x, int y);
  CellOccupied GetCell(Point2D pos);
  status::status Update(int x, int y, int update_value);
  status::status Update(Point2D pos, int update_value);
  std::vector<Point2D> GetLineLow(int x0, int y0, int x1, int y1);
  std::vector<Point2D> GetLineHigh(int x0, int y0, int x1, int y1);
  std::vector<Point2D> GetLine(int x0, int y0, int x1, int y1);
  status::status UpdateWithScanPoint(float x0, float y0, float x1, float y1,
                                     int update_value);
  int GetMapSizeXMin() { return 0; }
  int GetMapSizeXMax() { return map_.info.width; }
  int GetMapSizeYMin() { return 0; }
  int GetMapSizeYMax() { return map_.info.height; }

  Point2D TransformIndex(float x, float y);

  // only for simulation
  status::status Load(std::string path_to_map);
  status::status LoadGlobalMap(std::string path_to_map);

  // each time the probability should be changed by this value, plus or minus
  // if the input data is filtered, then the update_value is lower
  const int update_value_filtered = 6;
  // if the input data is original without filtering, then the update_value is
  // is higher
  const int update_value_full = 30;

private:
  // save the map data.
  nav_msgs::OccupancyGrid map_;

  // if the probability of this grid is bigger than occupied_bound,
  // then this cell will be treated as occupied
  int occupied_bound = 75;
  // if the probability of this grid is smaller than empty_bound,
  // then this cell will be treated as empty
  int empty_bound = 25;
};

#endif // MAP_H

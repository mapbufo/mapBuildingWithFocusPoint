#ifndef COMMON_H
#define COMMON_H
#include <algorithm>
#include <boost/unordered_map.hpp>
#include <iostream>
#include <math.h>
#include <utility>
#include <vector>

namespace status
{
  enum status
  {
    Error = 0,
    Ok = 1,
    Undifined = 2
  };

}

enum CellOccupied
{
  empty = 0,
  occupied = 1,
  unknown = 2,
  grey = 3,
  path = 4,
  robot_pos = 5,
  target_pos = 6
};
typedef std::pair<int, int> Point2D;
typedef std::pair<float, float> Point2DWithFloat;
typedef boost::unordered_map<Point2D, CellOccupied> ScanData;
typedef boost::unordered_map<Point2DWithFloat, int> ScanPointsFloatWithUpdateValue;

#endif // !COMMON_H

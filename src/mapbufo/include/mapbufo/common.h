#ifndef COMMON_H
#define COMMON_H
#include <math.h>
#include <algorithm>
#include <boost/unordered_map.hpp>
#include <iostream>
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

void transformPoint2DToPoint2DWithFloat(Point2D point2D, Point2DWithFloat &point2DWithFloat)
{
  point2DWithFloat.first = (float)(point2D.first);
  point2DWithFloat.second = (float)(point2D.second);
}

/**
 * transform the position from float real position to int grid position
 * @param input float x, float y: position of robot(real data)
 * @param input float res: resolution
 * @param return  Point2D: position of robot(grid position)
 */
Point2D TransformIndex(float x, float y, float res)
{
  float factor = 1 / res;
  Point2D pos;
  if (x >= 0)
  {
    pos.first = std::ceil(x * factor);
  }
  pos.first = std::floor(x * factor);
  if (y >= 0)
  {
    pos.second = std::ceil(y * factor);
  }
  pos.second = std::floor(y * factor);
  return pos;
}

Point2DWithFloat ReverseIndex(int x, int y, float res)
{
  Point2DWithFloat pos;
  pos.first = static_cast<float>(x * res);
  pos.second = static_cast<float>(y * res);
  return pos;
}
#endif // !COMMON_H

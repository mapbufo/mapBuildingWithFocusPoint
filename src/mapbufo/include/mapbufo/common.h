#ifndef COMMON_H
#define COMMON_H
#include <math.h>
#include <Eigen/Dense>
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
 * @brief transform the position from float real position to int grid position
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

/**
   * get the line-points with gradient between 0 and 1
   * @param input int x0, int y0: position of robot
   * @param input int x1, int y1: position of target point
   * @return vector<Point2D>: all the passed points in the line
   */
std::vector<Point2D> GetLineLow(int x0, int y0, int x1, int y1)
{
  int d_x = x1 - x0;
  int d_y = y1 - y0;
  int y_step = 1;
  if (d_y < 0)
  {
    y_step = -1;
    d_y = -d_y;
  }
  int D = 2 * d_y - d_x;
  int x = x0;
  int y = y0;
  std::vector<Point2D> res;
  Point2D curr({x, y});
  while (x < x1)
  {
    if (D > 0)
    {
      y += y_step;
      D -= 2 * d_x;
    }
    D += 2 * d_y;
    x += 1;
    curr.first = x;
    curr.second = y;
    res.insert(begin(res), curr);
  }
  res.erase(begin(res));
  return res;
}

/**
   * get the line-points with gradient greater than 1
   * @param input int x0, int y0: position of robot
   * @param input int x1, int y1: position of target point
   * @return vector<Point2D>: all the passed points in the line
   */
std::vector<Point2D> GetLineHigh(int x0, int y0, int x1, int y1)
{
  int d_x = x1 - x0;
  int d_y = y1 - y0;
  int x_step = 1;
  if (d_x < 0)
  {
    x_step = -1;
    d_x = -d_x;
  }
  int D = 2 * d_x - d_y;
  int x = x0;
  int y = y0;
  std::vector<Point2D> res;
  Point2D curr({x, y});
  while (y < y1)
  {
    if (D > 0)
    {
      x += x_step;
      D -= 2 * d_y;
    }
    D += 2 * d_x;
    y += 1;
    curr.first = x;
    curr.second = y;
    res.insert(begin(res), curr);
  }
  res.erase(begin(res));
  return res;
}

/**
  * get the line-points between robot and target point
  * @param input int x0, int y0: position of robot
  * @param input int x1, int y1: position of target point
  * @return vector<Point2D>: all the passed points in the line
  */
std::vector<Point2D> GetLine(int x0, int y0, int x1, int y1)
{
  if (abs(y1 - y0) < abs(x1 - x0))
  {
    if (x0 > x1)
    {
      return GetLineLow(x1, y1, x0, y0);
    }
    return GetLineLow(x0, y0, x1, y1);
  }
  if (y0 > y1)
  {
    return GetLineHigh(x1, y1, x0, y0);
  }
  return GetLineHigh(x0, y0, x1, y1);
}

Point2DWithFloat TransformFromGlobalToLocal(float robot_x, float robot_y, float target_x, float target_y, float yaw)
{
  Eigen::Matrix2f transform_matrix;
  transform_matrix << cos(yaw), -sin(yaw), sin(yaw), cos(yaw);
  Eigen::Vector2f global_point_vec(target_x, target_y);
  Eigen::Vector2f local_coord_vec(robot_x, robot_y);
  Eigen::Vector2f local_point_vec;

  // calculate the point_pos in local coordinate
  local_point_vec = transform_matrix.inverse() * (global_point_vec - local_coord_vec);

  return Point2DWithFloat(local_point_vec[0], local_point_vec[1]);
}

Point2DWithFloat TransformFromLocalToGlobal(float robot_x, float robot_y, float target_x, float target_y, float yaw)
{
  float global_x = target_x * std::cos(yaw) - target_y * std::sin(yaw) + robot_x;
  float global_y = target_x * std::sin(yaw) + target_y * std::cos(yaw) + robot_y;

  return Point2DWithFloat(global_x, global_y);
}

#endif // !COMMON_H

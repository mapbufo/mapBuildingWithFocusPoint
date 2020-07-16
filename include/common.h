#ifndef COMMON_H
#define COMMON_H
#include <boost/unordered_map.hpp>
#include <utility>
namespace status {
enum status { Error = 0, Ok = 1, Undifined = 2 };
}

enum CellOccupied {
  empty = 0,
  occupied = 1,
  unknown = 2,
  out_of_map = 3,
  path = 4,
  robot_pos = 5,
  target_pos = 6
};
typedef std::pair<int, int> Point2D;
typedef boost::unordered_map<Point2D, CellOccupied> ScanData;

#endif  // !COMMON_H

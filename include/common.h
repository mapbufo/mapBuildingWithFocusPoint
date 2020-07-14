#ifndef COMMON_H
#define COMMON_H
#include <utility>

namespace status {
enum status { Error = 0, Ok = 1, Undifined = 2 };
}

typedef std::pair<int, int> Point2D;

#endif  // !COMMON_H